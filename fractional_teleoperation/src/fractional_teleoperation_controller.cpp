#include "fractional_teleoperation/fractional_teleoperation_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace fractional_teleoperation_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
	// Declare robot_description parameter - will be provided by controller_manager
	auto node = get_node();
	if (!node->has_parameter("robot_description"))
	{
		node->declare_parameter("robot_description", rclcpp::ParameterValue(std::string("")));
	}

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

	declare_and_get_parameters("alpha", alpha_, 1.5);
	declare_and_get_parameters("fractional_offset_gain", gain_K_, 0.1);
	declare_and_get_parameters("memory_length", memory_length_, 100);
	declare_and_get_parameters("dt", dt_, 0.01);
	declare_and_get_parameters("output_velocity_scale", velocity_scale_, 1.0);
	declare_and_get_parameters(
			"global_linear_velocity_saturation", global_linear_velocity_saturation_, 0.0);
	declare_and_get_parameters(
			"global_angular_velocity_saturation", global_angular_velocity_saturation_, 0.0);
	declare_and_get_parameters("input_frame", input_frame_, std::string("base"));

	declare_and_get_parameters("normalize_gain_for_alpha", adapt_gain_to_alpha_, true);
	declare_and_get_parameters(
			"alpha_gain_normalization_mode", adaptive_gain_mode_, std::string("dt"));
	declare_and_get_parameters("v_max", v_max_, 1.0);
	declare_and_get_parameters("t_ref", t_ref_, 1.0);
	declare_and_get_parameters("k_0", k_0_, 1.0);
	declare_and_get_parameters("k_1", k_1_, 1.0);

	declare_and_get_parameters("use_reference_drift", use_reference_drift_, true);
	declare_and_get_parameters("reference_first_order_rate", reference_drift_rate_, 0.15);
	declare_and_get_parameters(
			"reference_drift_joystick_threshold", reference_drift_joystick_threshold_, 0.0);
	declare_and_get_parameters("joystick_active_threshold", joystick_active_threshold_, 0.01);
	declare_and_get_parameters("snap_reference_on_release", snap_reference_on_release_, false);
	declare_and_get_parameters("reference_update_mode", reference_update_mode_, std::string("first_order"));
	declare_and_get_parameters("reference_fractional_alpha", reference_alpha_, 0.8);
	declare_and_get_parameters("reference_fractional_gain", reference_fractional_gain_, 0.15);
	declare_and_get_parameters("linear_offset_scale_max", fractional_offset_linear_scale_max_, 1.0);
	declare_and_get_parameters("angular_offset_scale_max", fractional_offset_angular_scale_max_, 1.0);
	declare_and_get_parameters("offset_ramp_time", fractional_offset_scale_ramp_time_, 0.5);
	declare_and_get_parameters(
			"offset_ramp_profile", fractional_offset_scale_ramp_profile_, std::string("sigmoid"));

	declare_and_get_parameters("use_dynamic_alpha", use_dynamic_alpha_, false);
	declare_and_get_parameters("alpha_min", alpha_min_, 0.0);
	declare_and_get_parameters("alpha_max", alpha_max_, 1.0);
	declare_and_get_parameters("l_0", l_0_, 0.1);
	declare_and_get_parameters("l_max", l_max_, 1.0);
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

	if (alpha_ <= 0.0 || alpha_ >= 2.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid alpha=%.3f. Clamping to [0.1, 1.9].",
				alpha_);
		alpha_ = std::clamp(alpha_, 0.1, 1.9);
	}

	std::transform(
			adaptive_gain_mode_.begin(),
			adaptive_gain_mode_.end(),
			adaptive_gain_mode_.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (
			adaptive_gain_mode_ != "dt" && adaptive_gain_mode_ != "perceptual" &&
			adaptive_gain_mode_ != "geometric_transition")
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid alpha_gain_normalization_mode '%s'. Defaulting to 'dt'.",
				adaptive_gain_mode_.c_str());
		adaptive_gain_mode_ = "dt";
	}

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

	if (v_max_ <= 0.0)
	{
		v_max_ = 1.0;
	}
	if (t_ref_ <= 0.0)
	{
		t_ref_ = 1.0;
	}
	if (k_0_ <= 0.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid k_0=%.3f. Must be > 0. Using default 1.0.",
				k_0_);
		k_0_ = 1.0;
	}
	if (k_1_ <= 0.0)
	{
		RCLCPP_WARN(
				get_node()->get_logger(),
				"Invalid k_1=%.3f. Must be > 0. Using default 1.0.",
				k_1_);
		k_1_ = 1.0;
	}

	if (reference_alpha_ <= 0.0 || reference_alpha_ >= 2.0)
	{
		reference_alpha_ = std::clamp(reference_alpha_, 0.1, 1.9);
	}

	if (alpha_min_ < 0.0)
	{
		alpha_min_ = 0.0;
	}
	if (alpha_max_ >= 2.0)
	{
		alpha_max_ = 1.9;
	}
	if (alpha_min_ > alpha_max_)
	{
		alpha_min_ = alpha_max_;
	}

	if (global_linear_velocity_saturation_ < 0.0)
	{
		global_linear_velocity_saturation_ = 0.0;
	}
	if (global_angular_velocity_saturation_ < 0.0)
	{
		global_angular_velocity_saturation_ = 0.0;
	}

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

	current_alpha_ = use_dynamic_alpha_ ? alpha_min_ : alpha_;
	last_alpha_ = current_alpha_;
	updateGainK();

	gl_coefficients_ =
			fractional_teleoperation::core::computeGrunwaldCoefficients(memory_length_, current_alpha_);
	reference_gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
			memory_length_, reference_alpha_);

	latest_joystick_ = geometry_msgs::msg::Twist();
	mode_ = extender_msgs::msg::TeleopCommand::BOTH;
	desired_position_linear_.setZero();
	desired_position_angular_.setZero();
	reference_position_linear_.setZero();
	reference_position_angular_.setZero();
	current_orientation_ = Eigen::Quaterniond::Identity();
	linear_history_.clear();
	angular_history_.clear();
	reference_linear_history_.clear();
	reference_angular_history_.clear();
	joystick_was_active_ = false;
	joystick_active_duration_ = 0.0;
	last_linear_scale_ = 0.0;
	last_angular_scale_ = 0.0;
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
	if (!adapt_gain_to_alpha_)
	{
		return;
	}

	gain_K_ = fractional_teleoperation::core::computeAdaptiveGainK(
			current_alpha_,
			v_max_,
			t_ref_,
			dt_,
			k_0_,
			k_1_,
			adaptive_gain_mode_);
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
	const auto ee_pose = robot_vel_interface_->getCurrentEndEffectorPose();
	reference_position_linear_ = ee_pose.translation;
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

	visualization_msgs::msg::Marker marker_msg;
	marker_msg.header.stamp = get_node()->now();
	marker_msg.header.frame_id = marker_frame_id_;
	marker_msg.ns = "vel_cmd";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::msg::Marker::ARROW;
	marker_msg.action = visualization_msgs::msg::Marker::ADD;
	marker_msg.scale.x = marker_scale_x_;
	marker_msg.scale.y = marker_scale_y_;
	marker_msg.scale.z = marker_scale_z_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 0.0;
	marker_msg.color.b = 0.0;

	geometry_msgs::msg::Point start;
	geometry_msgs::msg::Point end;
	start.x = desired_position.x();
	start.y = desired_position.y();
	start.z = desired_position.z();
	end.x = start.x + vel_cmd.linear[0];
	end.y = start.y + vel_cmd.linear[1];
	end.z = start.z + vel_cmd.linear[2];

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

	visualization_msgs::msg::Marker marker_msg;
	marker_msg.header.stamp = get_node()->now();
	marker_msg.header.frame_id = marker_frame_id_;
	marker_msg.ns = "desired_position";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
	marker_msg.action = visualization_msgs::msg::Marker::ADD;
	marker_msg.scale.x = desired_position_marker_scale_;
	marker_msg.scale.y = desired_position_marker_scale_;
	marker_msg.scale.z = desired_position_marker_scale_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 0.3;
	marker_msg.color.b = 1.0;
	marker_msg.pose.position.x = desired_position.x();
	marker_msg.pose.position.y = desired_position.y();
	marker_msg.pose.position.z = desired_position.z();
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

	visualization_msgs::msg::Marker marker_msg;
	marker_msg.header.stamp = get_node()->now();
	marker_msg.header.frame_id = marker_frame_id_;
	marker_msg.ns = "reference_position";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
	marker_msg.action = visualization_msgs::msg::Marker::ADD;
	marker_msg.scale.x = reference_position_marker_scale_;
	marker_msg.scale.y = reference_position_marker_scale_;
	marker_msg.scale.z = reference_position_marker_scale_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 0.5;
	marker_msg.color.b = 0.0;
	marker_msg.pose.position.x = reference_position.x();
	marker_msg.pose.position.y = reference_position.y();
	marker_msg.pose.position.z = reference_position.z();
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

	visualization_msgs::msg::Marker marker_msg;
	marker_msg.header.stamp = get_node()->now();
	marker_msg.header.frame_id = marker_frame_id_;
	marker_msg.ns = "joystick_linear";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::msg::Marker::ARROW;
	marker_msg.action = visualization_msgs::msg::Marker::ADD;
	marker_msg.scale.x = marker_scale_x_;
	marker_msg.scale.y = marker_scale_y_;
	marker_msg.scale.z = marker_scale_z_;
	marker_msg.color.a = 1.0;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;

	geometry_msgs::msg::Point start;
	geometry_msgs::msg::Point end;
	start.x = reference_position.x();
	start.y = reference_position.y();
	start.z = reference_position.z();
	end.x = start.x + joystick_linear.x();
	end.y = start.y + joystick_linear.y();
	end.z = start.z + joystick_linear.z();

	marker_msg.points = {start, end};
	joystick_linear_marker_pub_->publish(marker_msg);
}

controller_interface::return_type FractionalTeleoperationController::update(
		const rclcpp::Time &,
		const rclcpp::Duration &)
{
	if (!robot_vel_interface_)
	{
		return controller_interface::return_type::ERROR;
	}

	robot_vel_interface_->syncState();

	const auto ee_pose = robot_vel_interface_->getCurrentEndEffectorPose();
	current_orientation_ = ee_pose.quaternion;

	using Mode = extender_msgs::msg::TeleopCommand;

	Eigen::Vector3d joystick_linear(
			latest_joystick_.linear.x,
			latest_joystick_.linear.y,
			latest_joystick_.linear.z);
	Eigen::Vector3d joystick_angular(
			latest_joystick_.angular.x,
			latest_joystick_.angular.y,
			latest_joystick_.angular.z);

	RCLCPP_DEBUG(
			get_node()->get_logger(),
			"joystick input - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
			joystick_linear.x(),
			joystick_linear.y(),
			joystick_linear.z(),
			joystick_angular.x(),
			joystick_angular.y(),
			joystick_angular.z());

	

	const Eigen::Vector3d previous_reference_position_linear = reference_position_linear_;
	const Eigen::Vector3d previous_reference_position_angular = reference_position_angular_;

	fractional_teleoperation::core::updateDynamicAlphaAndCoefficients(
			use_dynamic_alpha_,
			joystick_linear,
			l_0_,
			l_max_,
			alpha_min_,
			alpha_max_,
			alpha_threshold_,
			current_alpha_,
			last_alpha_,
			memory_length_,
			gl_coefficients_);
	if (adapt_gain_to_alpha_)
	{
		updateGainK();
	}

	Eigen::Vector3d previous_scaled_offset_linear = desired_position_linear_ * last_linear_scale_;
	Eigen::Vector3d previous_scaled_offset_angular = desired_position_angular_ * last_angular_scale_;
	const Eigen::Vector3d previous_desired_position_linear =
			previous_reference_position_linear + previous_scaled_offset_linear;
	const Eigen::Vector3d previous_desired_position_angular =
			previous_reference_position_angular + previous_scaled_offset_angular;
	const Eigen::Vector3d pre_update_offset_linear = desired_position_linear_;
	const Eigen::Vector3d pre_update_offset_angular = desired_position_angular_;

	desired_position_linear_ = fractional_teleoperation::core::applyFractionalIntegration(
			joystick_linear,
			linear_history_,
			gl_coefficients_,
			memory_length_,
			dt_,
			current_alpha_,
			gain_K_);
			
	desired_position_angular_ = fractional_teleoperation::core::applyFractionalIntegration(
			joystick_angular,
			angular_history_,
			gl_coefficients_,
			memory_length_,
			dt_,
			current_alpha_,
			gain_K_);

	const bool joystick_active = joystick_linear.norm() > joystick_active_threshold_;
	if (joystick_active)
	{
		joystick_active_duration_ += dt_;
	}
	else
	{
		joystick_active_duration_ = 0.0;
	}

	const double t_ratio = (fractional_offset_scale_ramp_time_ > 0.0)
														 ? std::min(1.0, joystick_active_duration_ / fractional_offset_scale_ramp_time_)
														 : 1.0;
	const double ramp_factor =
			fractional_teleoperation::ramp::computeRampFactor(t_ratio, fractional_offset_scale_ramp_profile_);
	const double current_linear_scale = fractional_offset_linear_scale_max_ * ramp_factor;
	const double current_angular_scale = fractional_offset_angular_scale_max_ * ramp_factor;

	Eigen::Vector3d scaled_offset_linear = desired_position_linear_ * current_linear_scale;
	Eigen::Vector3d scaled_offset_angular = desired_position_angular_ * current_angular_scale;

	Eigen::Vector3d desired_linear = reference_position_linear_ + scaled_offset_linear;
	Eigen::Vector3d desired_angular = reference_position_angular_ + scaled_offset_angular;

	if (snap_reference_on_release_ && joystick_was_active_ && !joystick_active)
	{
		reference_position_linear_ += pre_update_offset_linear * last_linear_scale_;
		reference_position_angular_ += pre_update_offset_angular * last_angular_scale_;
		desired_position_linear_.setZero();
		desired_position_angular_.setZero();
		linear_history_.clear();
		angular_history_.clear();
		previous_scaled_offset_linear.setZero();
		previous_scaled_offset_angular.setZero();
	}
	joystick_was_active_ = joystick_active;
	last_linear_scale_ = current_linear_scale;
	last_angular_scale_ = current_angular_scale;

	Eigen::Vector3d reference_linear_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d reference_angular_velocity = Eigen::Vector3d::Zero();
	if (use_reference_drift_)
	{
		reference_linear_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_linear_,
				previous_reference_position_linear,
				dt_);
		reference_angular_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
			reference_position_angular_,
				previous_reference_position_angular,
				dt_);
	}

	Eigen::Vector3d cartesian_linear_velocity =
			fractional_teleoperation::core::computeVelocityFromDesiredPosition(
					scaled_offset_linear,
					previous_scaled_offset_linear,
					dt_);
	Eigen::Vector3d cartesian_angular_velocity =
			fractional_teleoperation::core::computeVelocityFromDesiredPosition(
					scaled_offset_angular,
					previous_scaled_offset_angular,
					dt_);

	cartesian_linear_velocity -= reference_linear_velocity;
	cartesian_angular_velocity -= reference_angular_velocity;

	fractional_teleoperation::core::transformApplyModeAndScale(
			cartesian_linear_velocity,
			cartesian_angular_velocity,
			input_frame_,
			current_orientation_,
			mode_,
			Mode::TRANSLATION,
			Mode::ROTATION,
			velocity_scale_);

	if (global_linear_velocity_saturation_ > 0.0)
	{
		const double linear_velocity_norm = cartesian_linear_velocity.norm();
		if (linear_velocity_norm > global_linear_velocity_saturation_)
		{
			const double linear_saturation_scale = global_linear_velocity_saturation_ / linear_velocity_norm;
			cartesian_linear_velocity *= linear_saturation_scale;

			desired_linear = previous_desired_position_linear +
					linear_saturation_scale * (desired_linear - previous_desired_position_linear);

			scaled_offset_linear = desired_linear - reference_position_linear_;
			if (current_linear_scale > 1e-9)
			{
				desired_position_linear_ = scaled_offset_linear / current_linear_scale;
			}
			else
			{
				desired_position_linear_.setZero();
			}
		}
	}

	if (global_angular_velocity_saturation_ > 0.0)
	{
		const double angular_velocity_norm = cartesian_angular_velocity.norm();
		if (angular_velocity_norm > global_angular_velocity_saturation_)
		{
			const double angular_saturation_scale = global_angular_velocity_saturation_ / angular_velocity_norm;
			cartesian_angular_velocity *= angular_saturation_scale;

			desired_angular = previous_desired_position_angular +
					angular_saturation_scale * (desired_angular - previous_desired_position_angular);
			reference_position_angular_ = previous_reference_position_angular +
					angular_saturation_scale *
							(reference_position_angular_ - previous_reference_position_angular);

			scaled_offset_angular = desired_angular - reference_position_angular_;
			if (current_angular_scale > 1e-9)
			{
				desired_position_angular_ = scaled_offset_angular / current_angular_scale;
			}
			else
			{
				desired_position_angular_.setZero();
			}
		}
	}

	if (use_reference_drift_ && joystick_linear.norm() >= reference_drift_joystick_threshold_)
	{
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
		else
		{
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
	}

	robot_interfaces::CartesianVelocity latest_vel_cmd;
	latest_vel_cmd.linear[0] = cartesian_linear_velocity.x();
	latest_vel_cmd.linear[1] = cartesian_linear_velocity.y();
	latest_vel_cmd.linear[2] = cartesian_linear_velocity.z();
	latest_vel_cmd.angular[0] = cartesian_angular_velocity.x();
	latest_vel_cmd.angular[1] = cartesian_angular_velocity.y();
	latest_vel_cmd.angular[2] = cartesian_angular_velocity.z();

	RCLCPP_DEBUG(get_node()->get_logger(),
		"latest_vel_cmd - linear: [%.6f, %.6f, %.6f], angular: [%.6f, %.6f, %.6f]",
		latest_vel_cmd.linear[0], latest_vel_cmd.linear[1], latest_vel_cmd.linear[2],
		latest_vel_cmd.angular[0], latest_vel_cmd.angular[1], latest_vel_cmd.angular[2]);

	if (enable_marker_visualization_)
	{
		publishDesiredPositionMarker(desired_linear);
		publishReferencePositionMarker(reference_position_linear_);
		publishJoystickLinearMarker(joystick_linear, reference_position_linear_);
		publishMarker(latest_vel_cmd, desired_linear);
	}

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

}  // namespace fractional_teleoperation_controller

PLUGINLIB_EXPORT_CLASS(
		fractional_teleoperation_controller::FractionalTeleoperationController,
		controller_interface::ControllerInterface)
