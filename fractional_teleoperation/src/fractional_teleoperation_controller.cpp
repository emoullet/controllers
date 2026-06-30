#include "fractional_teleoperation/fractional_teleoperation_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "signal_processing/dead_zone.hpp"
#include "signal_processing/saturation.hpp"

namespace fractional_teleoperation_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{
geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &value)
{
	geometry_msgs::msg::Point point;
	point.x = value.x();
	point.y = value.y();
	point.z = value.z();
	return point;
}

visualization_msgs::msg::Marker createMarkerBase(
		const rclcpp::Time &stamp,
		const std::string &frame_id,
		const std::string &ns,
		int32_t type)
{
	visualization_msgs::msg::Marker marker_msg;
	marker_msg.header.stamp = stamp;
	marker_msg.header.frame_id = frame_id;
	marker_msg.ns = ns;
	marker_msg.id = 0;
	marker_msg.type = type;
	marker_msg.action = visualization_msgs::msg::Marker::ADD;
	return marker_msg;
}

robot_interfaces::CartesianVelocity toCartesianVelocity(
		const Eigen::Vector3d &linear,
		const Eigen::Vector3d &angular)
{
	robot_interfaces::CartesianVelocity vel_cmd;
	vel_cmd.linear[0] = linear.x();
	vel_cmd.linear[1] = linear.y();
	vel_cmd.linear[2] = linear.z();
	vel_cmd.angular[0] = angular.x();
	vel_cmd.angular[1] = angular.y();
	vel_cmd.angular[2] = angular.z();
	return vel_cmd;
}

Eigen::Quaterniond normalizedQuaternion(const Eigen::Quaterniond &quaternion)
{
	Eigen::Quaterniond normalized = quaternion;
	if (normalized.norm() < 1e-12)
	{
		return Eigen::Quaterniond::Identity();
	}
	normalized.normalize();
	return normalized;
}

Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d &rotation_vector)
{
	const double angle = rotation_vector.norm();
	if (angle < 1e-12)
	{
		return Eigen::Quaterniond::Identity();
	}

	return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond &quaternion)
{
	Eigen::Quaterniond normalized = normalizedQuaternion(quaternion);
	if (normalized.w() < 0.0)
	{
		normalized.coeffs() *= -1.0;
	}

	const Eigen::Vector3d vector_part = normalized.vec();
	const double sin_half_angle = vector_part.norm();
	if (sin_half_angle < 1e-12)
	{
		return 2.0 * vector_part;
	}

	const double angle = 2.0 * std::atan2(sin_half_angle, normalized.w());
	return angle * vector_part / sin_half_angle;
}

// Eigen::Vector3d getCurrentAngularVelocityFromRobotInterface()
// {
// 	if (!robot_vel_interface_)
// 	{
// 		return Eigen::Vector3d::Zero();
// 	}

	// const robot_interfaces::CartesianVelocity current_velocity = robot_vel_interface_->getCurrentCartesianVelocity();
	// return Eigen::Vector3d(
	// 		current_velocity.angular[0],
	// 		current_velocity.angular[1],
	// 		current_velocity.angular[2]);
// }

Eigen::Vector3d computeAngularVelocityFromOrientationError(
		const Eigen::Quaterniond &desired_orientation,
		const Eigen::Quaterniond &current_orientation,
		const Eigen::Vector3d &current_angular_velocity,
		const double Kp,
		const double Kd,
		double dt)
{
	if (dt <= 1e-12)
	{
		return Eigen::Vector3d::Zero();
	}
	
	Eigen::Quaterniond desired_orientation_flipped = desired_orientation;
	// Make quaternion sign consistent to avoid discontinuities

	// if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
	// {
	// 	Eigen::Quaterniond desired_orientation_flipped = Eigen::Quaterniond(-desired_orientation.w(), -desired_orientation.x(), -desired_orientation.y(), -desired_orientation.z());
	// }

	const Eigen::Quaterniond orientation_error_quat_ =
			normalizedQuaternion(desired_orientation_flipped) * normalizedQuaternion(current_orientation).conjugate();
	// if (orientation_error.norm() < 1)
	// {
	// 	std::cerr << "Warning: orientation error quaternion norm is less than 1. This may indicate an issue with the orientation computation." << std::endl;
	// 	return Eigen::Vector3d::Zero();
	// }
	const Eigen::Vector3d orientation_error_ = quaternionToRotationVector(orientation_error_quat_);
	const Eigen::Vector3d omega_desired_ = orientation_error_/dt;
	const Eigen::Vector3d angular_velocity_command_ = Kp * orientation_error_ + Kd * (omega_desired_ - current_angular_velocity);
	return angular_velocity_command_;
}
}  // namespace

FractionalTeleoperationController::FractionalTeleoperationController()
: controller_interface::ControllerInterface()
{
}

FractionalTeleoperationController::~FractionalTeleoperationController() = default;

controller_interface::InterfaceConfiguration
FractionalTeleoperationController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration config;

	if (use_topic_output_)
	{
		config.type = controller_interface::interface_configuration_type::NONE;
		return config;
	}

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	if (robot_vel_interface_)
	{
		config.names = robot_vel_interface_->get_commands_names();
	}
	else
	{
		config.names = command_names_;
	}

	return config;
}

controller_interface::InterfaceConfiguration
FractionalTeleoperationController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration config;
	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	if (robot_vel_interface_)
	{
		config.names = robot_vel_interface_->get_states_names();
	}

	return config;
}

CallbackReturn FractionalTeleoperationController::on_init()
{
	loadParameters();
	return CallbackReturn::SUCCESS;
}

void FractionalTeleoperationController::loadParameters()
{
	declareParameters();
	validateAndNormalizeParameters();
	resetControllerState();
}

void FractionalTeleoperationController::declareParameters()
{
	// Declare robot_description parameter - will be provided by controller_manager
	auto node = get_node();
	if (!node->has_parameter("robot_description"))
	{
		node->declare_parameter("robot_description", rclcpp::ParameterValue(std::string("")));
	}

	// get update_rate parameter from controller_manager
	declare_and_get_parameters("update_rate", update_rate_, 100);
	dt_ = double(1.0) / double(update_rate_);

	declare_and_get_parameters("robot_type", robot_type_, std::string("explorer_velocity"));
	declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
	declare_and_get_parameters("use_topic_output", use_topic_output_, false);
	declare_and_get_parameters(
			"vel_cmd_output_topic",
			vel_cmd_output_topic_,
			std::string("/fractional_teleoperation_controller/velocity_command"));
	declare_and_get_parameters("base_frame", base_frame_, std::string("base_link"));
	declare_and_get_parameters("tool_frame", tool_frame_, std::string("tool0"));
	declare_and_get_parameters("teleop_cmd_input_topic", teleop_cmd_input_topic_, std::string("/teleop_cmd"));

	declare_and_get_parameters("linear_alpha", linear_alpha_, 1.5);
	declare_and_get_parameters("angular_alpha", angular_alpha_, 1.5);
	declare_and_get_parameters("linear_fractional_offset_gain", linear_gain_K_, 0.1);
	declare_and_get_parameters("angular_fractional_offset_gain", angular_gain_K_, 0.1);
	declare_and_get_parameters("memory_length", memory_length_, 100);
	// declare_and_get_parameters("dt", dt_, 0.01);
	declare_and_get_parameters("output_velocity_scale", velocity_scale_, 1.0);
	declare_and_get_parameters(
			"global_linear_velocity_saturation", global_linear_velocity_saturation_, 0.0);
	declare_and_get_parameters(
			"global_angular_velocity_saturation", global_angular_velocity_saturation_, 0.0);
	declare_and_get_parameters(
			"enable_linear_one_euro_filter", enable_linear_one_euro_filter_, false);
	declare_and_get_parameters(
			"linear_one_euro_frequency",
			linear_one_euro_frequency_,
			static_cast<double>(update_rate_));
	declare_and_get_parameters("linear_one_euro_mincutoff", linear_one_euro_mincutoff_, 1.0);
	declare_and_get_parameters("linear_one_euro_beta", linear_one_euro_beta_, 0.1);
	declare_and_get_parameters("linear_one_euro_dcutoff", linear_one_euro_dcutoff_, 1.0);
	declare_and_get_parameters(
			"enable_angular_one_euro_filter", enable_angular_one_euro_filter_, false);
	declare_and_get_parameters(
			"angular_one_euro_frequency",
			angular_one_euro_frequency_,
			static_cast<double>(update_rate_));
	declare_and_get_parameters("angular_one_euro_mincutoff", angular_one_euro_mincutoff_, 1.0);
	declare_and_get_parameters("angular_one_euro_beta", angular_one_euro_beta_, 0.1);
	declare_and_get_parameters("angular_one_euro_dcutoff", angular_one_euro_dcutoff_, 1.0);
	declare_and_get_parameters("input_frame", input_frame_, std::string("base"));

	declare_and_get_parameters("linear_normalize_gain_for_alpha", linear_normalize_gain_for_alpha_, true);
	declare_and_get_parameters("angular_normalize_gain_for_alpha", angular_normalize_gain_for_alpha_, true);
	declare_and_get_parameters(
			"linear_alpha_gain_normalization_mode",
			linear_alpha_gain_normalization_mode_,
			std::string("dt"));
	declare_and_get_parameters(
			"angular_alpha_gain_normalization_mode",
			angular_alpha_gain_normalization_mode_,
			std::string("dt"));
	declare_and_get_parameters("linear_v_max", linear_v_max_, 1.0);
	declare_and_get_parameters("angular_v_max", angular_v_max_, 1.0);
	declare_and_get_parameters("linear_t_ref", linear_t_ref_, 1.0);
	declare_and_get_parameters("angular_t_ref", angular_t_ref_, 1.0);
	declare_and_get_parameters("linear_k_0", linear_k_0_, 1.0);
	declare_and_get_parameters("angular_k_0", angular_k_0_, 1.0);
	declare_and_get_parameters("linear_k_1", linear_k_1_, 1.0);
	declare_and_get_parameters("angular_k_1", angular_k_1_, 1.0);

	declare_and_get_parameters("use_reference_drift", use_reference_drift_, true);
	declare_and_get_parameters("reference_first_order_rate", reference_drift_rate_, 0.15);
	declare_and_get_parameters(
			"reference_drift_joystick_threshold", reference_drift_joystick_threshold_, 0.0);
	declare_and_get_parameters("joystick_active_threshold", joystick_active_threshold_, 0.01);
	declare_and_get_parameters("joystick_timeout_sec", joystick_timeout_sec_, 0.2);
	declare_and_get_parameters("linear_deadband", linear_deadband_, 0.01);
	declare_and_get_parameters("angular_deadband", angular_deadband_, 0.03);
	declare_and_get_parameters("kp_orient", kp_orient_, 1.0);
	declare_and_get_parameters("kd_orient", kd_orient_, 0.1);
	declare_and_get_parameters("snap_reference_on_release", snap_reference_on_release_, false);
	declare_and_get_parameters("reference_update_mode", reference_update_mode_, std::string("first_order"));
	declare_and_get_parameters("reference_fractional_alpha", reference_alpha_, 0.8);
	declare_and_get_parameters("reference_fractional_gain", reference_fractional_gain_, 0.15);
	declare_and_get_parameters("linear_offset_scale_max", fractional_offset_linear_scale_max_, 1.0);
	declare_and_get_parameters("angular_offset_scale_max", fractional_offset_angular_scale_max_, 1.0);
	declare_and_get_parameters("offset_ramp_time", fractional_offset_scale_ramp_time_, 0.5);
	declare_and_get_parameters(
			"offset_ramp_profile", fractional_offset_scale_ramp_profile_, std::string("sigmoid"));

	declare_and_get_parameters("use_dynamic_alpha_linear", use_dynamic_alpha_linear_, false);
	declare_and_get_parameters("use_dynamic_alpha_angular", use_dynamic_alpha_angular_, false);
	declare_and_get_parameters("linear_alpha_min", linear_alpha_min_, 0.0);
	declare_and_get_parameters("linear_alpha_max", linear_alpha_max_, 1.0);
	declare_and_get_parameters("linear_l_0", linear_l_0_, 0.1);
	declare_and_get_parameters("linear_l_max", linear_l_max_, 1.0);
	declare_and_get_parameters("angular_alpha_min", angular_alpha_min_, 0.0);
	declare_and_get_parameters("angular_alpha_max", angular_alpha_max_, 1.0);
	declare_and_get_parameters("angular_l_0", angular_l_0_, 0.1);
	declare_and_get_parameters("angular_l_max", angular_l_max_, 1.0);
	declare_and_get_parameters("alpha_threshold", alpha_threshold_, 0.001);

	declare_and_get_parameters("enable_marker_visualization", enable_marker_visualization_, true);
	declare_and_get_parameters("vel_cmd_marker_topic", marker_topic_, std::string("/vel_cmd_marker"));
	declare_and_get_parameters("vel_cmd_marker_frame_id", marker_frame_id_, std::string("base_link"));
	declare_and_get_parameters("vel_cmd_marker_scale_x", marker_scale_x_, 0.03);
	declare_and_get_parameters("vel_cmd_marker_scale_y", marker_scale_y_, 0.07);
	declare_and_get_parameters("vel_cmd_marker_scale_z", marker_scale_z_, 0.07);
	declare_and_get_parameters(
			"desired_position_marker_topic", desired_position_marker_topic_, std::string("/desired_position_marker"));
	declare_and_get_parameters("desired_position_marker_scale", desired_position_marker_scale_, 0.04);
	declare_and_get_parameters(
			"reference_position_marker_topic",
			reference_position_marker_topic_,
			std::string("/reference_position_marker"));
	declare_and_get_parameters(
			"reference_position_marker_scale", reference_position_marker_scale_, 0.04);
	declare_and_get_parameters(
			"joystick_linear_marker_topic", joystick_linear_marker_topic_, std::string("/joystick_linear_marker"));
	declare_and_get_parameters(
			"ee_pose_translation_marker_topic",
			ee_pose_translation_marker_topic_,
			std::string("/ee_pose_translation_marker"));
	declare_and_get_parameters(
			"ee_pose_translation_marker_scale", ee_pose_translation_marker_scale_, 0.04);
}

void FractionalTeleoperationController::validateAndNormalizeParameters()
{
	const auto clamp_fixed_alpha = [this](const char * parameter_name, double & alpha)
	{
		if (alpha <= 0.0 || alpha >= 2.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s=%.3f. Clamping to [0.1, 1.9].",
					parameter_name,
					alpha);
			alpha = std::clamp(alpha, 0.1, 1.9);
		}
	};

	clamp_fixed_alpha("linear_alpha", linear_alpha_);
	clamp_fixed_alpha("angular_alpha", angular_alpha_);

	const auto validate_gain_normalization_parameters =
			[this](
					const char * label,
					std::string & mode,
					double & v_max,
					double & t_ref,
					double & k_0,
					double & k_1)
	{
		std::transform(
				mode.begin(),
				mode.end(),
				mode.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (mode != "dt" && mode != "perceptual" && mode != "geometric_transition")
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_alpha_gain_normalization_mode '%s'. Defaulting to 'dt'.",
					label,
					mode.c_str());
			mode = "dt";
		}
		else
		{
			RCLCPP_INFO(
					get_node()->get_logger(),
					"Using %s_alpha_gain_normalization_mode '%s'.",
					label,
					mode.c_str());
		}

		if (v_max <= 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_v_max=%.3f. Must be > 0. Using default 1.0.",
					label,
					v_max);
			v_max = 1.0;
		}
		if (t_ref <= 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_t_ref=%.3f. Must be > 0. Using default 1.0.",
					label,
					t_ref);
			t_ref = 1.0;
		}
		if (k_0 <= 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_k_0=%.3f. Must be > 0. Using default 1.0.",
					label,
					k_0);
			k_0 = 1.0;
		}
		if (k_1 < 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_k_1=%.3f. Must be >= 0. Using default 1.0.",
					label,
					k_1);
			k_1 = 1.0;
		}
	};

	validate_gain_normalization_parameters(
			"linear",
			linear_alpha_gain_normalization_mode_,
			linear_v_max_,
			linear_t_ref_,
			linear_k_0_,
			linear_k_1_);
	validate_gain_normalization_parameters(
			"angular",
			angular_alpha_gain_normalization_mode_,
			angular_v_max_,
			angular_t_ref_,
			angular_k_0_,
			angular_k_1_);

	std::transform(
			input_frame_.begin(),
			input_frame_.end(),
			input_frame_.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (input_frame_ != "base" && input_frame_ != "ee")
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid input_frame '%s'. Defaulting to 'base'.",
				input_frame_.c_str());
		input_frame_ = "base";
	}

	std::transform(
			reference_update_mode_.begin(),
			reference_update_mode_.end(),
			reference_update_mode_.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (reference_update_mode_ != "first_order" && reference_update_mode_ != "fractional")
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid reference_update_mode '%s'. Defaulting to 'first_order'.",
				reference_update_mode_.c_str());
		reference_update_mode_ = "first_order";
	}

	if (reference_alpha_ <= 0.0 || reference_alpha_ >= 2.0)
	{
		reference_alpha_ = std::clamp(reference_alpha_, 0.1, 1.9);
	}

	if (joystick_timeout_sec_ < 0.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid joystick_timeout_sec=%.3f. Clamping to 0.0.",
				joystick_timeout_sec_);
		joystick_timeout_sec_ = 0.0;
	}
	if (joystick_timeout_sec_ > 5.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"joystick_timeout_sec=%.3f too large. Clamping to 5.0.",
				joystick_timeout_sec_);
		joystick_timeout_sec_ = 5.0;
	}

	const auto clamp_deadband =
			[this](const char * parameter_name, double & deadband)
	{
		if (deadband < 0.0 || deadband > 0.9)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s=%.3f. Clamping to [0.0, 0.9].",
					parameter_name,
					deadband);
			deadband = std::clamp(deadband, 0.0, 0.9);
		}
	};
	clamp_deadband("linear_deadband", linear_deadband_);
	clamp_deadband("angular_deadband", angular_deadband_);


	if (kp_orient_ < 0.0 || kp_orient_ > 10.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid kp_orient=%.3f. Clamping to [0.0, 10.0].",
				kp_orient_);
		kp_orient_ = std::clamp(kp_orient_, 0.0, 10.0);
	}
	if (kd_orient_ < 0.0 || kd_orient_ > 10.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid kd_orient=%.3f. Clamping to [0.0, 10.0].",
				kd_orient_);
		kd_orient_ = std::clamp(kd_orient_, 0.0, 10.0);
	}

	const auto normalize_dynamic_alpha_range =
			[this](const char * label, double & alpha_min, double & alpha_max, double & l_0, double & l_max)
	{
		alpha_min = std::clamp(alpha_min, 0.0, 1.9);
		alpha_max = std::clamp(alpha_max, 0.0, 1.9);
		if (alpha_min > alpha_max)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"%s alpha_min %.3f is greater than alpha_max %.3f. Setting alpha_min=alpha_max.",
					label,
					alpha_min,
					alpha_max);
			alpha_min = alpha_max;
		}
		if (l_0 < 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"%s l_0 %.3f is negative. Clamping to 0.0.",
					label,
					l_0);
			l_0 = 0.0;
		}
		if (l_max <= l_0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"%s l_max %.3f must be greater than l_0 %.3f. Setting l_max=l_0+1.0.",
					label,
					l_max,
					l_0);
			l_max = l_0 + 1.0;
		}
	};

	normalize_dynamic_alpha_range(
			"linear dynamic alpha",
			linear_alpha_min_,
			linear_alpha_max_,
			linear_l_0_,
			linear_l_max_);
	normalize_dynamic_alpha_range(
			"angular dynamic alpha",
			angular_alpha_min_,
			angular_alpha_max_,
			angular_l_0_,
			angular_l_max_);

	if (global_linear_velocity_saturation_ < 0.0)
	{
		global_linear_velocity_saturation_ = 0.0;
	}
	if (global_angular_velocity_saturation_ < 0.0)
	{
		global_angular_velocity_saturation_ = 0.0;
	}

	const auto validate_one_euro_parameters =
			[this](
					const char * label,
					double & frequency,
					double & mincutoff,
					double & beta,
					double & dcutoff)
	{
		if (!std::isfinite(frequency) || frequency < 1e-3)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_one_euro_frequency=%.6f. Clamping to 1e-3.",
					label,
					frequency);
			frequency = 1e-3;
		}
		if (!std::isfinite(mincutoff) || mincutoff < 1e-6)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_one_euro_mincutoff=%.6f. Clamping to 1e-6.",
					label,
					mincutoff);
			mincutoff = 1e-6;
		}
		if (!std::isfinite(beta) || beta < 0.0)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_one_euro_beta=%.6f. Clamping to 0.0.",
					label,
					beta);
			beta = 0.0;
		}
		if (!std::isfinite(dcutoff) || dcutoff < 1e-6)
		{
			RCLCPP_WARN(
					get_node()->get_logger(),
					"Invalid %s_one_euro_dcutoff=%.6f. Clamping to 1e-6.",
					label,
					dcutoff);
			dcutoff = 1e-6;
		}
	};
	validate_one_euro_parameters(
			"linear",
			linear_one_euro_frequency_,
			linear_one_euro_mincutoff_,
			linear_one_euro_beta_,
			linear_one_euro_dcutoff_);
	validate_one_euro_parameters(
			"angular",
			angular_one_euro_frequency_,
			angular_one_euro_mincutoff_,
			angular_one_euro_beta_,
			angular_one_euro_dcutoff_);

	if (fractional_offset_linear_scale_max_ < 0.0)
	{
		fractional_offset_linear_scale_max_ = 0.0;
	}
	if (fractional_offset_angular_scale_max_ < 0.0)
	{
		fractional_offset_angular_scale_max_ = 0.0;
	}
	if (fractional_offset_scale_ramp_time_ < 0.0)
	{
		fractional_offset_scale_ramp_time_ = 0.0;
	}

	try
	{
		fractional_offset_scale_ramp_profile_ =
				fractional_teleoperation::ramp::validateProfileName(fractional_offset_scale_ramp_profile_);
	}
	catch (const std::invalid_argument &)
	{
		fractional_offset_scale_ramp_profile_ = "sigmoid";
	}
}


void FractionalTeleoperationController::resetControllerState()
{
	current_linear_alpha_ = use_dynamic_alpha_linear_ ? linear_alpha_min_ : linear_alpha_;
	current_angular_alpha_ = use_dynamic_alpha_angular_ ? angular_alpha_min_ : angular_alpha_;
	last_linear_alpha_ = current_linear_alpha_;
	last_angular_alpha_ = current_angular_alpha_;
	updateGainK();

	linear_gl_coefficients_ =
			fractional_teleoperation::core::computeGrunwaldCoefficients(memory_length_, current_linear_alpha_);
	angular_gl_coefficients_ =
			fractional_teleoperation::core::computeGrunwaldCoefficients(memory_length_, current_angular_alpha_);
	reference_gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
			memory_length_, reference_alpha_);

	linear_velocity_filter_ = signal_processing::OneEuroFilter3d(
			linear_one_euro_frequency_,
			linear_one_euro_mincutoff_,
			linear_one_euro_beta_,
			linear_one_euro_dcutoff_);
	angular_velocity_filter_ = signal_processing::OneEuroFilter3d(
			angular_one_euro_frequency_,
			angular_one_euro_mincutoff_,
			angular_one_euro_beta_,
			angular_one_euro_dcutoff_);
	linear_velocity_filter_.reset();
	angular_velocity_filter_.reset();

	latest_joystick_ = geometry_msgs::msg::Twist();
	mode_ = extender_msgs::msg::TeleopCommand::BOTH;
	desired_offset_linear_.setZero();
	desired_offset_angular_.setZero();
	reference_position_linear_.setZero();
	reference_position_angular_.setZero();
	current_orientation_ = Eigen::Quaterniond::Identity();
	reference_orientation_ = Eigen::Quaterniond::Identity();
	linear_history_.clear();
	angular_history_.clear();
	reference_linear_history_.clear();
	reference_angular_history_.clear();
	joystick_was_active_ = false;
	joystick_active_duration_ = 0.0;
	last_linear_scale_ = 0.0;
	last_angular_scale_ = 0.0;
	joystick_message_received_ = false;
	last_joystick_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void FractionalTeleoperationController::setupSubscribers()
{
	joystick_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
			teleop_cmd_input_topic_,
			10,
			std::bind(&FractionalTeleoperationController::joystickCallback, this, std::placeholders::_1));

	// Subscribe to robot_description topic as fallback if parameter is not available
	robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
			"/robot_description",
			rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
			std::bind(&FractionalTeleoperationController::robotDescriptionCallback, this, std::placeholders::_1));
}

void FractionalTeleoperationController::declarePublishers()
{
	if (use_topic_output_)
	{
		auto node = get_node();
		vel_cmd_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
				vel_cmd_output_topic_,
				10);
	}

	if (!enable_marker_visualization_)
	{
		return;
	}

	auto node = get_node();
	vel_cmd_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
	desired_position_marker_pub_ =
			node->create_publisher<visualization_msgs::msg::Marker>(desired_position_marker_topic_, 10);
	reference_position_marker_pub_ =
			node->create_publisher<visualization_msgs::msg::Marker>(reference_position_marker_topic_, 10);
	joystick_linear_marker_pub_ =
			node->create_publisher<visualization_msgs::msg::Marker>(joystick_linear_marker_topic_, 10);
	ee_pose_translation_marker_pub_ =
			node->create_publisher<visualization_msgs::msg::Marker>(ee_pose_translation_marker_topic_, 10);
}

void FractionalTeleoperationController::activatePublishers()
{
	if (!enable_marker_visualization_)
	{
		return;
	}

	vel_cmd_marker_pub_->on_activate();
	desired_position_marker_pub_->on_activate();
	reference_position_marker_pub_->on_activate();
	joystick_linear_marker_pub_->on_activate();
	ee_pose_translation_marker_pub_->on_activate();
}

void FractionalTeleoperationController::deactivatePublishers()
{
	if (!enable_marker_visualization_)
	{
		return;
	}

	vel_cmd_marker_pub_->on_deactivate();
	desired_position_marker_pub_->on_deactivate();
	reference_position_marker_pub_->on_deactivate();
	joystick_linear_marker_pub_->on_deactivate();
	ee_pose_translation_marker_pub_->on_deactivate();
}

bool FractionalTeleoperationController::setupRobotInterface()
{
	std::string robot_description;
	auto node = get_node();

	// Try to get robot_description from parameter first
	if (node->has_parameter("robot_description"))
	{
		if (node->get_parameter("robot_description", robot_description) && !robot_description.empty())
		{
			RCLCPP_INFO(node->get_logger(), "Got robot_description from parameter");
		}
	}

	// If not available via parameter, use cached version from topic subscription
	if (robot_description.empty() && !cached_robot_description_.empty())
	{
		robot_description = cached_robot_description_;
		RCLCPP_INFO(node->get_logger(), "Got robot_description from /robot_description topic");
	}

	// If still empty, wait a bit and retry (controller_manager may publish it shortly)
	if (robot_description.empty())
	{
		RCLCPP_WARN(node->get_logger(),
			"robot_description not yet available. Waiting for /robot_description topic...");
		
		// Give the topic subscription a moment to receive the message
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		
		if (!cached_robot_description_.empty())
		{
			robot_description = cached_robot_description_;
			RCLCPP_INFO(node->get_logger(), "Successfully got robot_description from /robot_description topic");
		}
	}

	if (robot_description.empty())
	{
		RCLCPP_ERROR(node->get_logger(),
			"Failed to get robot_description. Make sure robot_state_publisher is running and publishing to /robot_description topic.");
		return false;
	}

	robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type_);
	if (!robot_vel_interface_ ||
		  !robot_vel_interface_->initKinematics(robot_description, tool_frame_))
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
		return false;
	}

	robot_vel_interface_->set_commands_names(command_names_);
	return true;
}

void FractionalTeleoperationController::updateGainK()
{
	if (linear_normalize_gain_for_alpha_)
	{
		linear_gain_K_ = fractional_teleoperation::core::computeAdaptiveGainK(
				current_linear_alpha_,
				linear_v_max_,
				linear_t_ref_,
				dt_,
				linear_k_0_,
				linear_k_1_,
				linear_alpha_gain_normalization_mode_);
	}
	if (angular_normalize_gain_for_alpha_)
	{
		angular_gain_K_ = fractional_teleoperation::core::computeAdaptiveGainK(
				current_angular_alpha_,
				angular_v_max_,
				angular_t_ref_,
				dt_,
				angular_k_0_,
				angular_k_1_,
				angular_alpha_gain_normalization_mode_);
	}
	RCLCPP_INFO(
			get_node()->get_logger(),
			"Current gains: linear_gain_K=%.3f (alpha=%.3f, normalized=%s), angular_gain_K=%.3f (alpha=%.3f, normalized=%s)",
			linear_gain_K_,
			current_linear_alpha_,
			linear_normalize_gain_for_alpha_ ? "true" : "false",
			angular_gain_K_,
			current_angular_alpha_,
			angular_normalize_gain_for_alpha_ ? "true" : "false");
}

void FractionalTeleoperationController::readJoystickInput(
		Eigen::Vector3d &joystick_linear,
		Eigen::Vector3d &joystick_angular) const
{
	joystick_linear = Eigen::Vector3d(
			latest_joystick_.linear.x,
			latest_joystick_.linear.y,
			latest_joystick_.linear.z);
	joystick_angular = Eigen::Vector3d(
			latest_joystick_.angular.x,
			latest_joystick_.angular.y,
			latest_joystick_.angular.z);
}

Eigen::Vector3d FractionalTeleoperationController::applyDeadbandToVector(
		const Eigen::Vector3d &input,
		double deadband) const
{
	return signal_processing::applyScaledDeadZonePerAxis(input, deadband, 1.0);
}

bool FractionalTeleoperationController::isJoystickInputFresh() const
{
	if (!joystick_message_received_)
	{
		return false;
	}

	if (joystick_timeout_sec_ <= 0.0)
	{
		return true;
	}

	return (get_node()->now() - last_joystick_stamp_).seconds() <= joystick_timeout_sec_;
}

void FractionalTeleoperationController::integrateDesiredOffsets(
		const Eigen::Vector3d &joystick_linear,
		const Eigen::Vector3d &joystick_angular)
{
	RCLCPP_INFO(
			get_node()->get_logger(),
			"Integrating desired offuuuuuuuuuusets with linear_alpha=%.3f, linear_gain_K=%.3f, angular_alpha=%.3f, angular_gain_K=%.3f",
			current_linear_alpha_,
			linear_gain_K_,
			current_angular_alpha_,
			angular_gain_K_);
	desired_offset_linear_ = fractional_teleoperation::core::applyFractionalIntegration(
			joystick_linear,
			linear_history_,
			linear_gl_coefficients_,
			memory_length_,
			dt_,
			current_linear_alpha_,
			linear_gain_K_);
	RCLCPP_INFO(get_node()->get_logger(), "Desired offset linear: [%.3f, %.3f, %.3f]", desired_offset_linear_.x(), desired_offset_linear_.y(), desired_offset_linear_.z());

	desired_offset_angular_ = fractional_teleoperation::core::applyFractionalIntegration(
			joystick_angular,
			angular_history_,
			angular_gl_coefficients_,
			memory_length_,
			dt_,
			current_angular_alpha_,
			angular_gain_K_);
}

void FractionalTeleoperationController::updateJoystickActivityDuration(bool joystick_active)
{
	if (joystick_active)
	{
		joystick_active_duration_ += dt_;
	}
	else
	{
		joystick_active_duration_ = 0.0;
	}
}

void FractionalTeleoperationController::applySnapOnRelease(
		bool joystick_active,
		const Eigen::Vector3d &pre_update_offset_linear,
		const Eigen::Vector3d &pre_update_offset_angular,
		Eigen::Vector3d &previous_scaled_offset_linear,
		Eigen::Vector3d &previous_scaled_offset_angular)
{
	if (snap_reference_on_release_ && joystick_was_active_ && !joystick_active)
	{
		reference_position_linear_ += pre_update_offset_linear * last_linear_scale_;
		reference_position_angular_ += pre_update_offset_angular * last_angular_scale_;
		desired_offset_linear_.setZero();
		desired_offset_angular_.setZero();
		linear_history_.clear();
		angular_history_.clear();
		previous_scaled_offset_linear.setZero();
		previous_scaled_offset_angular.setZero();
	}

	joystick_was_active_ = joystick_active;
}

void FractionalTeleoperationController::applyLinearVelocitySaturation(
		Eigen::Vector3d &cartesian_linear_velocity,
		const Eigen::Vector3d &previous_desired_position_linear,
		Eigen::Vector3d &desired_linear,
		double current_linear_scale,
		Eigen::Vector3d &scaled_offset_linear)
{
	if (global_linear_velocity_saturation_ <= 0.0)
	{
		return;
	}

	const double linear_velocity_norm = cartesian_linear_velocity.norm();
	if (linear_velocity_norm <= global_linear_velocity_saturation_)
	{
		return;
	}

	const double linear_saturation_scale = global_linear_velocity_saturation_ / linear_velocity_norm;
	cartesian_linear_velocity =
			signal_processing::limitNorm(cartesian_linear_velocity, global_linear_velocity_saturation_);

	desired_linear = previous_desired_position_linear +
			linear_saturation_scale * (desired_linear - previous_desired_position_linear);

	scaled_offset_linear = desired_linear - reference_position_linear_;
	if (current_linear_scale > 1e-9)
	{
		desired_offset_linear_ = scaled_offset_linear / current_linear_scale;
	}
	else
	{
		desired_offset_linear_.setZero();
	}
}

void FractionalTeleoperationController::applyAngularVelocitySaturation(
		Eigen::Vector3d &cartesian_angular_velocity,
		const Eigen::Vector3d &previous_desired_position_angular,
		const Eigen::Vector3d &previous_reference_position_angular,
		Eigen::Vector3d &desired_angular,
		double current_angular_scale,
		Eigen::Vector3d &scaled_offset_angular)
{
	if (global_angular_velocity_saturation_ <= 0.0)
	{
		return;
	}

	const double angular_velocity_norm = cartesian_angular_velocity.norm();
	if (angular_velocity_norm <= global_angular_velocity_saturation_)
	{
		return;
	}

	const double angular_saturation_scale = global_angular_velocity_saturation_ / angular_velocity_norm;
	cartesian_angular_velocity =
			signal_processing::limitNorm(cartesian_angular_velocity, global_angular_velocity_saturation_);

	desired_angular = previous_desired_position_angular +
			angular_saturation_scale * (desired_angular - previous_desired_position_angular);
	reference_position_angular_ = previous_reference_position_angular +
			angular_saturation_scale *
					(reference_position_angular_ - previous_reference_position_angular);

	scaled_offset_angular = desired_angular - reference_position_angular_;
	if (current_angular_scale > 1e-9)
	{
		desired_offset_angular_ = scaled_offset_angular / current_angular_scale;
	}
	else
	{
		desired_offset_angular_.setZero();
	}
}

void FractionalTeleoperationController::computeReferenceDriftVelocity(
		const Eigen::Vector3d &previous_reference_position_linear,
		const Eigen::Vector3d &previous_reference_position_angular,
		Eigen::Vector3d &reference_linear_velocity,
		Eigen::Vector3d &reference_angular_velocity) const
{
	if (!use_reference_drift_)
	{
		reference_linear_velocity.setZero();
		reference_angular_velocity.setZero();
		return;
	}

	reference_linear_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_linear_,
			previous_reference_position_linear,
			dt_);
	reference_angular_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_angular_,
			previous_reference_position_angular,
			dt_);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> FractionalTeleoperationController::updateReferenceDrift(
		double joystick_linear_norm,
		const Eigen::Vector3d &desired_linear,
		const Eigen::Vector3d &desired_angular)
{
	if (!use_reference_drift_ || joystick_linear_norm < reference_drift_joystick_threshold_)
	{
		return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	}
	
	Eigen::Vector3d reference_linear_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d reference_angular_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d previous_reference_position_linear = reference_position_linear_;
	Eigen::Vector3d previous_reference_position_angular = reference_position_angular_;

	if (reference_update_mode_ == "fractional")
	{
		reference_position_linear_ = fractional_teleoperation::core::updateReferencePositionFractional(
				reference_position_linear_,
				desired_linear,
				reference_linear_history_,
				reference_gl_coefficients_,
				memory_length_,
				dt_,
				reference_alpha_,
				reference_fractional_gain_);
		reference_position_angular_ = fractional_teleoperation::core::updateReferencePositionFractional(
				reference_position_angular_,
				desired_angular,
				reference_angular_history_,
				reference_gl_coefficients_,
				memory_length_,
				dt_,
				reference_alpha_,
				reference_fractional_gain_);
	}
	else if (reference_update_mode_ == "first_order"){
		reference_position_linear_ = fractional_teleoperation::core::updateReferencePosition(
				reference_position_linear_,
				desired_linear,
				reference_drift_rate_,
				dt_);
		reference_position_angular_ = fractional_teleoperation::core::updateReferencePosition(
				reference_position_angular_,
				desired_angular,
				reference_drift_rate_,
				dt_);
	}
	else {
		RCLCPP_ERROR(get_node()->get_logger(), "Invalid reference_update_mode '%s'.", reference_update_mode_.c_str());
		return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	}
	reference_linear_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_linear_,
			previous_reference_position_linear,
			dt_);
	reference_angular_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_angular_,
			previous_reference_position_angular,
			dt_);
	
	return std::make_pair(reference_linear_velocity, reference_angular_velocity);

}

robot_interfaces::CartesianVelocity FractionalTeleoperationController::filterVelocityCommand(
		const robot_interfaces::CartesianVelocity &raw_vel_cmd,
		double timestamp_sec)
{
	Eigen::Vector3d linear_velocity(
			raw_vel_cmd.linear[0],
			raw_vel_cmd.linear[1],
			raw_vel_cmd.linear[2]);
	Eigen::Vector3d angular_velocity(
			raw_vel_cmd.angular[0],
			raw_vel_cmd.angular[1],
			raw_vel_cmd.angular[2]);

	if (enable_linear_one_euro_filter_)
	{
		linear_velocity = linear_velocity_filter_.filter(linear_velocity, timestamp_sec);
	}
	if (enable_angular_one_euro_filter_)
	{
		angular_velocity = angular_velocity_filter_.filter(angular_velocity, timestamp_sec);
	}

	return toCartesianVelocity(linear_velocity, angular_velocity);
}

controller_interface::return_type FractionalTeleoperationController::publishVelocityCommand(
		const robot_interfaces::CartesianVelocity &latest_vel_cmd)
{
	if (use_topic_output_)
	{
		if (!vel_cmd_pub_)
		{
			RCLCPP_ERROR(get_node()->get_logger(), "Velocity command publisher is not initialized.");
			return controller_interface::return_type::ERROR;
		}

		geometry_msgs::msg::TwistStamped msg;
		msg.header.stamp = get_node()->now();
		msg.header.frame_id = base_frame_;
		msg.twist.linear.x = latest_vel_cmd.linear[0];
		msg.twist.linear.y = latest_vel_cmd.linear[1];
		msg.twist.linear.z = latest_vel_cmd.linear[2];
		msg.twist.angular.x = latest_vel_cmd.angular[0];
		msg.twist.angular.y = latest_vel_cmd.angular[1];
		msg.twist.angular.z = latest_vel_cmd.angular[2];
		vel_cmd_pub_->publish(msg);
		return controller_interface::return_type::OK;
	}

	if (robot_vel_interface_->setCommand(latest_vel_cmd))
	{
		return controller_interface::return_type::OK;
	}

	RCLCPP_ERROR(get_node()->get_logger(), "Set command failed.");
	return controller_interface::return_type::ERROR;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> FractionalTeleoperationController::rampDesiredOffset(
		const Eigen::Vector3d &desired_offset_linear,
		const Eigen::Vector3d &desired_offset_angular,
		double joystick_active_duration,
		double &current_linear_scale,
		double &current_angular_scale) const
{
	const double t_ratio = (fractional_offset_scale_ramp_time_ > 0.0)
							   ? std::min(1.0, joystick_active_duration / fractional_offset_scale_ramp_time_)
							   : 1.0;
	const double ramp_factor =
		fractional_teleoperation::ramp::computeRampFactor(t_ratio, fractional_offset_scale_ramp_profile_);
	current_linear_scale = fractional_offset_linear_scale_max_ * ramp_factor;
	current_angular_scale = fractional_offset_angular_scale_max_ * ramp_factor;
	return {
		desired_offset_linear * current_linear_scale,
		desired_offset_angular * current_angular_scale};
}

CallbackReturn FractionalTeleoperationController::on_configure(const rclcpp_lifecycle::State &)
{
	loadParameters();
	setupSubscribers();
	declarePublishers();

	if (!setupRobotInterface())
	{
		return CallbackReturn::ERROR;
	}

	return CallbackReturn::SUCCESS;
}

CallbackReturn FractionalTeleoperationController::on_activate(const rclcpp_lifecycle::State &)
{
    if (!use_topic_output_)
    {
        robot_vel_interface_->assign_loaned_command(command_interfaces_);
    }
    robot_vel_interface_->assign_loaned_state(state_interfaces_);

    robot_vel_interface_->syncState();  // ensure fresh state before pose read
    const auto ee_pose = robot_vel_interface_->getCurrentEndEffectorPose();
    reference_position_linear_ = ee_pose.translation;
	current_orientation_ = normalizedQuaternion(ee_pose.quaternion);

		Eigen::Vector3d desired_orientation_vec = Eigen::Vector3d::Zero();
	// make x commponent pi/2
	desired_orientation_vec[1] = M_PI_2;
	reference_orientation_ = normalizedQuaternion(rotationVectorToQuaternion(desired_orientation_vec));
	// reference_orientation_ = current_orientation_;
	reference_position_angular_.setZero();
	linear_velocity_filter_.reset();
	angular_velocity_filter_.reset();

    RCLCPP_INFO(
            get_node()->get_logger(),
            "on_activate: initialized reference_position_linear to [%.6f, %.6f, %.6f]",
            reference_position_linear_.x(),
            reference_position_linear_.y(),
            reference_position_linear_.z());

    activatePublishers();
    return CallbackReturn::SUCCESS;
}

CallbackReturn FractionalTeleoperationController::on_deactivate(const rclcpp_lifecycle::State &)
{
	robot_vel_interface_->release_all_interfaces();
	deactivatePublishers();
	return CallbackReturn::SUCCESS;
}

void FractionalTeleoperationController::joystickCallback(
		const extender_msgs::msg::TeleopCommand::SharedPtr msg)
{
	latest_joystick_ = msg->twist;
	mode_ = msg->mode;
	last_joystick_stamp_ = get_node()->now();
	joystick_message_received_ = true;
}

void FractionalTeleoperationController::robotDescriptionCallback(
		const std_msgs::msg::String::SharedPtr msg)
{
	cached_robot_description_ = msg->data;
	RCLCPP_DEBUG(
		get_node()->get_logger(),
		"Received robot_description via /robot_description topic (%zu bytes)",
		cached_robot_description_.size());
}

void FractionalTeleoperationController::publishMarker(
		const robot_interfaces::CartesianVelocity &vel_cmd,
		const Eigen::Vector3d &desired_position)
{
	if (!enable_marker_visualization_ || !vel_cmd_marker_pub_)
	{
		return;
	}

	visualization_msgs::msg::Marker marker_msg = createMarkerBase(
			get_node()->now(), marker_frame_id_, "vel_cmd", visualization_msgs::msg::Marker::ARROW);
	marker_msg.scale.x = marker_scale_x_;
	marker_msg.scale.y = marker_scale_y_;
	marker_msg.scale.z = marker_scale_z_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 0.0;
	marker_msg.color.b = 0.0;

	const geometry_msgs::msg::Point start = toPoint(desired_position);
	geometry_msgs::msg::Point end = start;
	end.x += vel_cmd.linear[0];
	end.y += vel_cmd.linear[1];
	end.z += vel_cmd.linear[2];

	marker_msg.points = {start, end};
	vel_cmd_marker_pub_->publish(marker_msg);
}

void FractionalTeleoperationController::publishDesiredPositionMarker(
		const Eigen::Vector3d &desired_position)
{
	if (!enable_marker_visualization_ || !desired_position_marker_pub_)
	{
		return;
	}

	visualization_msgs::msg::Marker marker_msg = createMarkerBase(
			get_node()->now(),
			marker_frame_id_,
			"desired_position",
			visualization_msgs::msg::Marker::SPHERE);
	marker_msg.scale.x = desired_position_marker_scale_;
	marker_msg.scale.y = desired_position_marker_scale_;
	marker_msg.scale.z = desired_position_marker_scale_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 0.3;
	marker_msg.color.b = 1.0;
	marker_msg.pose.position = toPoint(desired_position);
	marker_msg.pose.orientation.w = 1.0;

	desired_position_marker_pub_->publish(marker_msg);
}

void FractionalTeleoperationController::publishReferencePositionMarker(
		const Eigen::Vector3d &reference_position)
{
	if (!enable_marker_visualization_ || !reference_position_marker_pub_)
	{
		return;
	}

	visualization_msgs::msg::Marker marker_msg = createMarkerBase(
			get_node()->now(),
			marker_frame_id_,
			"reference_position",
			visualization_msgs::msg::Marker::SPHERE);
	marker_msg.scale.x = reference_position_marker_scale_;
	marker_msg.scale.y = reference_position_marker_scale_;
	marker_msg.scale.z = reference_position_marker_scale_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 0.5;
	marker_msg.color.b = 0.0;
	marker_msg.pose.position = toPoint(reference_position);
	marker_msg.pose.orientation.w = 1.0;

	reference_position_marker_pub_->publish(marker_msg);
}

void FractionalTeleoperationController::publishJoystickLinearMarker(
		const Eigen::Vector3d &joystick_linear,
		const Eigen::Vector3d &reference_position)
{
	if (!enable_marker_visualization_ || !joystick_linear_marker_pub_)
	{
		return;
	}

	visualization_msgs::msg::Marker marker_msg = createMarkerBase(
			get_node()->now(), marker_frame_id_, "joystick_linear", visualization_msgs::msg::Marker::ARROW);
	marker_msg.scale.x = marker_scale_x_;
	marker_msg.scale.y = marker_scale_y_;
	marker_msg.scale.z = marker_scale_z_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;

	const geometry_msgs::msg::Point start = toPoint(reference_position);
	geometry_msgs::msg::Point end = start;
	end.x += joystick_linear.x();
	end.y += joystick_linear.y();
	end.z += joystick_linear.z();

	marker_msg.points = {start, end};
	joystick_linear_marker_pub_->publish(marker_msg);
}

void FractionalTeleoperationController::publishEePoseTranslationMarker(
		const Eigen::Vector3d &ee_pose_translation)
{
	if (!enable_marker_visualization_ || !ee_pose_translation_marker_pub_)
	{
		return;
	}

	visualization_msgs::msg::Marker marker_msg = createMarkerBase(
			get_node()->now(),
			marker_frame_id_,
			"ee_pose_translation",
			visualization_msgs::msg::Marker::SPHERE);
	marker_msg.scale.x = ee_pose_translation_marker_scale_;
	marker_msg.scale.y = ee_pose_translation_marker_scale_;
	marker_msg.scale.z = ee_pose_translation_marker_scale_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.2;
	marker_msg.pose.position = toPoint(ee_pose_translation);
	marker_msg.pose.orientation.w = 1.0;

	ee_pose_translation_marker_pub_->publish(marker_msg);
}


controller_interface::return_type FractionalTeleoperationController::update(
		const rclcpp::Time & time,
		const rclcpp::Duration & period)
{
	if (!robot_vel_interface_)
	{
		return controller_interface::return_type::ERROR;
	}
	
	robot_vel_interface_->syncState();
	// const double dt_ = period.seconds();
	
	const auto ee_pose = robot_vel_interface_->getCurrentEndEffectorPose();
	current_position_ = ee_pose.translation;
	current_orientation_ = normalizedQuaternion(ee_pose.quaternion);

	const robot_interfaces::CartesianVelocity current_velocity = robot_vel_interface_->getCurrentCartesianVelocity();
	current_angular_velocity_ = Eigen::Vector3d(
		current_velocity.angular[0],
		current_velocity.angular[1],
		current_velocity.angular[2]);

	Eigen::Vector3d joystick_linear;
	Eigen::Vector3d joystick_angular;
	readJoystickInput(joystick_linear, joystick_angular);
	// if (!isJoystickInputFresh())
	// {
	// 	joystick_linear.setZero();
	// 	joystick_angular.setZero();
	// }
	// else
	// {
	// 	joystick_linear = applyDeadbandToVector(joystick_linear, linear_deadband_);
	// 	joystick_angular = applyDeadbandToVector(joystick_angular, angular_deadband_);
	// }

	RCLCPP_DEBUG(
			get_node()->get_logger(),
			"joystick input - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
			joystick_linear.x(),
			joystick_linear.y(),
			joystick_linear.z(),
			joystick_angular.x(),
			joystick_angular.y(),
			joystick_angular.z());

	

	fractional_teleoperation::core::updateDynamicAlphaAndCoefficients(
			use_dynamic_alpha_linear_,
			joystick_linear,
			linear_l_0_,
			linear_l_max_,
			linear_alpha_min_,
			linear_alpha_max_,
			alpha_threshold_,
			current_linear_alpha_,
			last_linear_alpha_,
			memory_length_,
			linear_gl_coefficients_);
	fractional_teleoperation::core::updateDynamicAlphaAndCoefficients(
			use_dynamic_alpha_angular_,
			joystick_angular,
			angular_l_0_,
			angular_l_max_,
			angular_alpha_min_,
			angular_alpha_max_,
			alpha_threshold_,
			current_angular_alpha_,
			last_angular_alpha_,
			memory_length_,
			angular_gl_coefficients_);
		
	RCLCPP_INFO(
			get_node()->get_logger(),
			"Current azzzzzzzzzzzzlpha: linear=%.3f (norm=%.6f, l_0=%.3f, l_max=%.3f, alpha_min=%.3f, alpha_max=%.3f), angular=%.3f (norm=%.6f, l_0=%.3f, l_max=%.3f, alpha_min=%.3f, alpha_max=%.3f), alpha_threshold=%.6f",
			current_linear_alpha_,
			joystick_linear.norm(),
			linear_l_0_,
			linear_l_max_,
			linear_alpha_min_,
			linear_alpha_max_,
			current_angular_alpha_,
			joystick_angular.norm(),
			angular_l_0_,
			angular_l_max_,
			angular_alpha_min_,
			angular_alpha_max_,
			alpha_threshold_);
			
	if (linear_normalize_gain_for_alpha_ || angular_normalize_gain_for_alpha_)
	{
		updateGainK();
	}

	integrateDesiredOffsets(joystick_linear, joystick_angular);

	const double joystick_linear_norm = joystick_linear.norm();
	// const bool joystick_active = joystick_linear_norm > joystick_active_threshold_;
	// updateJoystickActivityDuration(joystick_active);

	// double current_linear_scale = 0.0;
	// double current_angular_scale = 0.0;
	// std::tie(desired_offset_linear_, desired_offset_angular_) = rampDesiredOffset(
	// 		desired_offset_linear_,
	// 		desired_offset_angular_,
	// 		joystick_active_duration_,
	// 		current_linear_scale,
	// 		current_angular_scale);

	Eigen::Vector3d desired_linear = reference_position_linear_ + desired_offset_linear_;
	Eigen::Vector3d desired_angular = reference_position_angular_ + desired_offset_angular_;

	// // applySnapOnRelease(
	// // 		joystick_active,
	// // 		pre_update_offset_linear,
	// // 		pre_update_offset_angular,
	// // 		previous_scaled_offset_linear,
	// // 		previous_scaled_offset_angular);
	// // last_linear_scale_ = current_linear_scale;
	// // last_angular_scale_ = current_angular_scale;

	Eigen::Vector3d reference_linear_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d reference_angular_velocity = Eigen::Vector3d::Zero();
	// computeReferenceDriftVelocity(
	// 		previous_reference_position_linear,
	// 		previous_reference_position_angular,
	// 		reference_linear_velocity,
	// 		reference_angular_velocity);

	

	Eigen::Vector3d cartesian_linear_velocity =
			fractional_teleoperation::core::computeVelocityFromDesiredPosition(
					desired_linear,
					current_position_,
					period.seconds());

	RCLCPP_INFO(get_node()->get_logger(),
		"Desired position - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
		desired_linear.x(),
		desired_linear.y(),
		desired_linear.z(),
		desired_angular.x(),
		desired_angular.y(),
		desired_angular.z());
	

	const Eigen::Quaterniond desired_orientation = normalizedQuaternion(rotationVectorToQuaternion(desired_offset_angular_) * reference_orientation_);

	Eigen::Vector3d cartesian_angular_velocity = computeAngularVelocityFromOrientationError(
			desired_orientation,
			current_orientation_,
			current_angular_velocity_,
			kp_orient_,
			kd_orient_,
			period.seconds());
	
			
		RCLCPP_INFO(get_node()->get_logger(),
		"cartesian velocity - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
		cartesian_linear_velocity.x(),
		cartesian_linear_velocity.y(),		
	cartesian_linear_velocity.z(),
		cartesian_angular_velocity.x(),
		cartesian_angular_velocity.y(),
		cartesian_angular_velocity.z());


	std::tie(reference_linear_velocity, reference_angular_velocity) = updateReferenceDrift(joystick_linear_norm, desired_linear, desired_angular);


	cartesian_linear_velocity -= reference_linear_velocity;
	// cartesian_angular_velocity -= reference_angular_velocity;

	const robot_interfaces::CartesianVelocity raw_vel_cmd =
			toCartesianVelocity(cartesian_linear_velocity, cartesian_angular_velocity);
	const robot_interfaces::CartesianVelocity latest_vel_cmd =
			filterVelocityCommand(raw_vel_cmd, time.seconds());

	RCLCPP_DEBUG(get_node()->get_logger(),
		"latest_vel_cmd - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
		latest_vel_cmd.linear[0], latest_vel_cmd.linear[1], latest_vel_cmd.linear[2],
		latest_vel_cmd.angular[0], latest_vel_cmd.angular[1], latest_vel_cmd.angular[2]);

	if (enable_marker_visualization_)
	{
		publishDesiredPositionMarker(desired_linear);
		publishReferencePositionMarker(reference_position_linear_);
		publishJoystickLinearMarker(joystick_linear, reference_position_linear_);
		publishEePoseTranslationMarker(ee_pose.translation);
		publishMarker(latest_vel_cmd, desired_linear);
	}

	return publishVelocityCommand(latest_vel_cmd);
}

}  // namespace fractional_teleoperation_controller

PLUGINLIB_EXPORT_CLASS(
		fractional_teleoperation_controller::FractionalTeleoperationController,
		controller_interface::ControllerInterface)
