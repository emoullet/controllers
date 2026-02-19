#include "fractional_teleoperation/fractional_teleoperation_controller.hpp"
#include "fractional_teleoperation/fractional_teleoperation_core.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace fractional_teleoperation_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FractionalTeleoperationController::FractionalTeleoperationController()
      : ControllerInterface(), latest_joystick_(),
        desired_position_linear_(Eigen::Vector3d::Zero()),
        desired_position_angular_(Eigen::Vector3d::Zero()),
        current_orientation_(Eigen::Quaterniond::Identity()),
        current_alpha_(1.5)
  {
  }

  FractionalTeleoperationController::~FractionalTeleoperationController() = default;

  controller_interface::InterfaceConfiguration FractionalTeleoperationController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    config.names = robot_vel_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration FractionalTeleoperationController::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_vel_interface_->get_states_names();
    return config;
  }

  CallbackReturn FractionalTeleoperationController::on_init()
  {
    // Create a subscription to the joystick command topic
    joystick_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&FractionalTeleoperationController::joystickCallback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  void FractionalTeleoperationController::loadParameters()
  {
    declare_and_get_parameters("alpha", alpha_, 1.5);
    declare_and_get_parameters("gain_K", gain_K_, 0.1);
    declare_and_get_parameters("memory_length", memory_length_, 100);
    declare_and_get_parameters("dt", dt_, 0.01);
    declare_and_get_parameters("velocity_scale", velocity_scale_, 1.0);
    declare_and_get_parameters("input_frame", input_frame_, std::string("base"));
    declare_and_get_parameters("robot_type", robot_type_, std::string("explorer_velocity"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
    declare_and_get_parameters("enable_marker_visualization", enable_marker_visualization_, true);
    declare_and_get_parameters("vel_cmd_marker_topic", marker_topic_, std::string("/vel_cmd_marker"));
    declare_and_get_parameters("vel_cmd_marker_frame_id", marker_frame_id_, std::string("base_link"));
    declare_and_get_parameters("vel_cmd_marker_scale_x", marker_scale_x_, 0.03);
    declare_and_get_parameters("vel_cmd_marker_scale_y", marker_scale_y_, 0.07);
    declare_and_get_parameters("vel_cmd_marker_scale_z", marker_scale_z_, 0.07);
    
    // Dynamic alpha parameters
    declare_and_get_parameters("use_dynamic_alpha", use_dynamic_alpha_, false);
    declare_and_get_parameters("alpha_max", alpha_max_, 1.0);
    declare_and_get_parameters("l_0", l_0_, 0.1);
    declare_and_get_parameters("l_max", l_max_, 1.0);
    
    // Initialize current_alpha_ based on use_dynamic_alpha
    if (use_dynamic_alpha_)
    {
      current_alpha_ = 0.0;  // Start with alpha=0
    }
    else
    {
      current_alpha_ = alpha_;  // Use fixed alpha
    }
    last_alpha_ = current_alpha_;
  }

  void FractionalTeleoperationController::declarePublishers()
  {
    if (enable_marker_visualization_) {
      vel_cmd_marker_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
      RCLCPP_INFO(get_node()->get_logger(), "Published vel_cmd marker on topic: %s", marker_topic_.c_str());
    }
  }

  bool FractionalTeleoperationController::setupRobotInterface()
  {
    auto node = get_node();
    std::string robot_description;

    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type_);
    if (!robot_vel_interface_ ||
        !robot_vel_interface_->initKinematics(robot_description,
                                              node->get_parameter("base_frame").as_string(),
                                              node->get_parameter("tool_frame").as_string()))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }
    robot_vel_interface_->set_commands_names(command_names_);

    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  FractionalTeleoperationController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    loadParameters();

    declarePublishers();

    // Validate alpha
    if (alpha_ <= 0.0 || alpha_ >= 2.0)
    {
      RCLCPP_ERROR(node->get_logger(), 
                   "Invalid fractional order alpha=%.3f. Must be in (0, 2). Clamping to [0.1, 1.9].",
                   alpha_);
      alpha_ = std::clamp(alpha_, 0.1, 1.9);
    }

    // Sanitize input frame
    std::transform(input_frame_.begin(), input_frame_.end(), input_frame_.begin(), ::tolower);
    if (input_frame_ != "base" && input_frame_ != "ee")
    {
      RCLCPP_WARN(node->get_logger(),
                  "Invalid input_frame '%s'. Use 'base' or 'ee'. Defaulting to 'base'.",
                  input_frame_.c_str());
      input_frame_ = "base";
    }

    // Logging
    RCLCPP_INFO(node->get_logger(), "Fractional-Order Teleoperation Controller:");
    RCLCPP_INFO(node->get_logger(), "  alpha (fractional order): %.4f", alpha_);
    RCLCPP_INFO(node->get_logger(), "  gain_K: %.4f", gain_K_);
    RCLCPP_INFO(node->get_logger(), "  memory_length: %d", memory_length_);
    RCLCPP_INFO(node->get_logger(), "  dt: %.6f s", dt_);
    RCLCPP_INFO(node->get_logger(), "  velocity_scale: %.4f", velocity_scale_);
    RCLCPP_INFO(node->get_logger(), "  input_frame: %s", input_frame_.c_str());
    
    // Log dynamic alpha parameters if enabled
    if (use_dynamic_alpha_)
    {
      RCLCPP_INFO(node->get_logger(), "  Dynamic Alpha Enabled:");
      RCLCPP_INFO(node->get_logger(), "    alpha_max: %.4f", alpha_max_);
      RCLCPP_INFO(node->get_logger(), "    l_0 (lower threshold): %.4f", l_0_);
      RCLCPP_INFO(node->get_logger(), "    l_max (upper threshold): %.4f", l_max_);
    }

    // Compute Grünwald-Letnikov coefficients
    gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
      memory_length_, current_alpha_);
    RCLCPP_INFO(node->get_logger(),
          "Computed %zu Grünwald-Letnikov coefficients for alpha=%.3f",
          gl_coefficients_.size(), current_alpha_);

    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }

    // Initialize history buffers
    linear_history_.clear();
    angular_history_.clear();
    desired_position_linear_.setZero();
    desired_position_angular_.setZero();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn FractionalTeleoperationController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the robot interface
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    robot_vel_interface_->assign_loaned_state(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn FractionalTeleoperationController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    
    // Clear history buffers
    linear_history_.clear();
    angular_history_.clear();
    desired_position_linear_.setZero();
    desired_position_angular_.setZero();
    
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type FractionalTeleoperationController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    typedef extender_msgs::msg::TeleopCommand Mode;
    
    // Get current EE pose
    robot_interfaces::CartesianPosition current_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();

    current_orientation_ = current_pose.quaternion;

    // Extract joystick input (normalized 3D)
    Eigen::Vector3d joystick_linear(latest_joystick_.linear.x, 
                                    latest_joystick_.linear.y,
                                    latest_joystick_.linear.z);
    Eigen::Vector3d joystick_angular(latest_joystick_.angular.x, 
                                     latest_joystick_.angular.y,
                                     latest_joystick_.angular.z);

    const double alpha_threshold = 0.001;
    fractional_teleoperation::core::updateDynamicAlphaAndCoefficients(
        use_dynamic_alpha_,
        joystick_linear,
        l_0_,
        l_max_,
        alpha_max_,
        alpha_threshold,
        current_alpha_,
        last_alpha_,
        memory_length_,
        gl_coefficients_);

    // Apply fractional-order differential law: D^alpha x_d(t) = K u(t)
    // This is implemented by integrating the fractional derivative numerically
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

    // Compute Cartesian velocities from desired positions
    Eigen::Vector3d cartesian_linear_velocity = 
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        desired_position_linear_, Eigen::Vector3d::Zero(), dt_); // Relative motion
    Eigen::Vector3d cartesian_angular_velocity = 
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        desired_position_angular_, Eigen::Vector3d::Zero(), dt_);

    fractional_teleoperation::core::transformApplyModeAndScale(
        cartesian_linear_velocity,
        cartesian_angular_velocity,
        input_frame_,
        current_orientation_,
        mode_,
        Mode::TRANSLATION,
        Mode::ROTATION,
        velocity_scale_);

    // Send command to robot
    robot_interfaces::CartesianVelocity vel_cmd;
    vel_cmd.linear = cartesian_linear_velocity;
    vel_cmd.angular = cartesian_angular_velocity;

    // Publish marker visualization for velocity command
    if (enable_marker_visualization_) {
      publishMarker(vel_cmd);
    }

    if (robot_vel_interface_->setCommand(vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  void FractionalTeleoperationController::joystickCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    // Update the stored joystick command with the latest message
    latest_joystick_ = msg->twist;
    mode_ = msg->mode;
  }

  void FractionalTeleoperationController::publishMarker(const robot_interfaces::CartesianVelocity &vel_cmd)
  {
    if (!enable_marker_visualization_ || !vel_cmd_marker_pub_) {
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
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    end.x = vel_cmd.linear.x();
    end.y = vel_cmd.linear.y();
    end.z = vel_cmd.linear.z();
    
    marker_msg.points = {start, end};
    vel_cmd_marker_pub_->publish(marker_msg);
  }

} // namespace fractional_teleoperation_controller

PLUGINLIB_EXPORT_CLASS(fractional_teleoperation_controller::FractionalTeleoperationController,
                       controller_interface::ControllerInterface)
