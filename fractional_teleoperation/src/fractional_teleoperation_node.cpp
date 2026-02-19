#include "fractional_teleoperation/fractional_teleoperation_node.hpp"
#include "fractional_teleoperation/fractional_teleoperation_core.hpp"

#include <algorithm>

namespace fractional_teleoperation_node
{
  FractionalTeleoperationNode::FractionalTeleoperationNode(
      const rclcpp::NodeOptions & options)
      : rclcpp::Node("fractional_teleoperation_node", options),
        latest_joystick_(),
        desired_position_linear_(Eigen::Vector3d::Zero()),
        desired_position_angular_(Eigen::Vector3d::Zero()),
        current_orientation_(Eigen::Quaterniond::Identity()),
        current_alpha_(1.5)
  {
    // Load all parameters
    loadParameters();

    // Declare subscribers and publishers
    declareSubscribers();
    declarePublishers();

    // Validate alpha
    if (alpha_ <= 0.0 || alpha_ >= 2.0)
    {
      RCLCPP_ERROR(get_logger(), 
                   "Invalid fractional order alpha=%.3f. Must be in (0, 2). Clamping to [0.1, 1.9].",
                   alpha_);
      alpha_ = std::clamp(alpha_, 0.1, 1.9);
    }

    // Validate adaptive gain mode
    std::transform(adaptive_gain_mode_.begin(), adaptive_gain_mode_.end(), adaptive_gain_mode_.begin(), ::tolower);
    if (adaptive_gain_mode_ != "dt" && adaptive_gain_mode_ != "perceptual")
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid adaptive_gain_mode '%s'. Use 'dt' or 'perceptual'. Defaulting to 'dt'.",
                  adaptive_gain_mode_.c_str());
      adaptive_gain_mode_ = "dt";
    }

    // Validate v_max
    if (v_max_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid v_max=%.3f. Must be > 0. Using default 1.0 m/s.",
                  v_max_);
      v_max_ = 1.0;
    }

    // Validate t_ref
    if (t_ref_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid t_ref=%.3f. Must be > 0. Using default 1.0 s.",
                  t_ref_);
      t_ref_ = 1.0;
    }

    // Recompute gain with validated parameters
    updateGainK();

    // Sanitize input frame
    std::transform(input_frame_.begin(), input_frame_.end(), input_frame_.begin(), ::tolower);
    if (input_frame_ != "base" && input_frame_ != "ee")
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid input_frame '%s'. Use 'base' or 'ee'. Defaulting to 'base'.",
                  input_frame_.c_str());
      input_frame_ = "base";
    }

    // Log configuration
    RCLCPP_INFO(get_logger(), "Fractional-Order Teleoperation Node:");
    RCLCPP_INFO(get_logger(), "  alpha (fractional order): %.4f", alpha_);
    RCLCPP_INFO(get_logger(), "  gain_K: %.6f", gain_K_);
    RCLCPP_INFO(get_logger(), "  memory_length: %d", memory_length_);
    RCLCPP_INFO(get_logger(), "  dt: %.6f s", dt_);
    RCLCPP_INFO(get_logger(), "  velocity_scale: %.4f", velocity_scale_);
    RCLCPP_INFO(get_logger(), "  input_frame: %s", input_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  adapt_gain_to_alpha: %s", adapt_gain_to_alpha_ ? "true" : "false");
    if (adapt_gain_to_alpha_)
    {
      RCLCPP_INFO(get_logger(), "  adaptive_gain_mode: %s", adaptive_gain_mode_.c_str());
      RCLCPP_INFO(get_logger(), "  v_max: %.3f m/s", v_max_);
      if (adaptive_gain_mode_ == "perceptual")
      {
        RCLCPP_INFO(get_logger(), "  t_ref: %.3f s", t_ref_);
      }
    }
    
    // Log dynamic alpha parameters if enabled
    if (use_dynamic_alpha_)
    {
      RCLCPP_INFO(get_logger(), "  Dynamic Alpha Enabled:");
      RCLCPP_INFO(get_logger(), "    alpha_max: %.4f", alpha_max_);
      RCLCPP_INFO(get_logger(), "    l_0 (lower threshold): %.4f", l_0_);
      RCLCPP_INFO(get_logger(), "    l_max (upper threshold): %.4f", l_max_);
    }

    // Compute Grünwald-Letnikov coefficients
    gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
      memory_length_, current_alpha_);

    // Initialize history buffers
    linear_history_.clear();
    angular_history_.clear();

    // Create update timer (runs at 1/dt frequency)
    auto update_period = std::chrono::milliseconds(static_cast<long>(dt_ * 1000));
    update_timer_ = create_wall_timer(
        update_period,
        std::bind(&FractionalTeleoperationNode::controlUpdate, this));

    RCLCPP_INFO(get_logger(), "Fractional teleoperation node initialized successfully.");
  }

  void FractionalTeleoperationNode::loadParameters()
  {
    declare_parameter("alpha", 1.5);
    declare_parameter("gain_K", 0.1);
    declare_parameter("memory_length", 100);
    declare_parameter("dt", 0.01);
    declare_parameter("velocity_scale", 1.0);
    declare_parameter("input_frame", std::string("base"));
    declare_parameter("adapt_gain_to_alpha", true);
    declare_parameter("adaptive_gain_mode", std::string("dt"));
    declare_parameter("v_max", 1.0);
    declare_parameter("t_ref", 1.0);
    declare_parameter("enable_marker_visualization", true);
    declare_parameter("vel_cmd_marker_topic", std::string("/vel_cmd_marker"));
    declare_parameter("vel_cmd_marker_frame_id", std::string("base_link"));
    declare_parameter("vel_cmd_marker_scale_x", 0.03);
    declare_parameter("vel_cmd_marker_scale_y", 0.07);
    declare_parameter("vel_cmd_marker_scale_z", 0.07);
    declare_parameter("desired_position_marker_topic", std::string("/desired_position_marker"));
    declare_parameter("desired_position_marker_scale", 0.04);
    declare_parameter("use_dynamic_alpha", false);
    declare_parameter("alpha_max", 1.0);
    declare_parameter("l_0", 0.1);
    declare_parameter("l_max", 1.0);
    declare_parameter("vel_cmd_topic", std::string("/vel_cmd"));

    alpha_ = get_parameter("alpha").as_double();
    gain_K_ = get_parameter("gain_K").as_double();
    memory_length_ = get_parameter("memory_length").as_int();
    dt_ = get_parameter("dt").as_double();
    velocity_scale_ = get_parameter("velocity_scale").as_double();
    input_frame_ = get_parameter("input_frame").as_string();
    adapt_gain_to_alpha_ = get_parameter("adapt_gain_to_alpha").as_bool();
    adaptive_gain_mode_ = get_parameter("adaptive_gain_mode").as_string();
    v_max_ = get_parameter("v_max").as_double();
    t_ref_ = get_parameter("t_ref").as_double();
    enable_marker_visualization_ = get_parameter("enable_marker_visualization").as_bool();
    marker_topic_ = get_parameter("vel_cmd_marker_topic").as_string();
    marker_frame_id_ = get_parameter("vel_cmd_marker_frame_id").as_string();
    marker_scale_x_ = get_parameter("vel_cmd_marker_scale_x").as_double();
    marker_scale_y_ = get_parameter("vel_cmd_marker_scale_y").as_double();
    marker_scale_z_ = get_parameter("vel_cmd_marker_scale_z").as_double();
    desired_position_marker_topic_ = get_parameter("desired_position_marker_topic").as_string();
    desired_position_marker_scale_ = get_parameter("desired_position_marker_scale").as_double();
    use_dynamic_alpha_ = get_parameter("use_dynamic_alpha").as_bool();
    alpha_max_ = get_parameter("alpha_max").as_double();
    l_0_ = get_parameter("l_0").as_double();
    l_max_ = get_parameter("l_max").as_double();

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
    updateGainK();
  }

  void FractionalTeleoperationNode::declareSubscribers()
  {
    joystick_sub_ = create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&FractionalTeleoperationNode::joystickCallback, this, std::placeholders::_1));
  }

  void FractionalTeleoperationNode::declarePublishers()
  {
    std::string vel_cmd_topic = get_parameter("vel_cmd_topic").as_string();
    vel_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(vel_cmd_topic, 10);
    RCLCPP_INFO(get_logger(), "Publishing vel_cmd on topic: %s", vel_cmd_topic.c_str());

    if (enable_marker_visualization_) {
      vel_cmd_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
      RCLCPP_INFO(get_logger(), "Publishing vel_cmd marker on topic: %s", marker_topic_.c_str());

      desired_position_marker_pub_ =
          create_publisher<visualization_msgs::msg::Marker>(desired_position_marker_topic_, 10);
      RCLCPP_INFO(get_logger(), "Publishing desired position marker on topic: %s",
                  desired_position_marker_topic_.c_str());
    }
  }

  void FractionalTeleoperationNode::updateGainK()
  {
    if (!adapt_gain_to_alpha_)
    {
      return;
    }

    if (adaptive_gain_mode_ == "perceptual" && current_alpha_ <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Perceptual gain mode requires alpha > 0. Using alpha=0.0010 for Gamma(alpha).");
    }

    gain_K_ = fractional_teleoperation::core::computeAdaptiveGainK(
        current_alpha_, v_max_, t_ref_, dt_, adaptive_gain_mode_);
  }

  void FractionalTeleoperationNode::joystickCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    // Update the stored joystick command with the latest message
    latest_joystick_ = msg->twist;
    mode_ = msg->mode;
  }

  void FractionalTeleoperationNode::publishMarker(const geometry_msgs::msg::Twist &vel_cmd)
  {
    if (!enable_marker_visualization_ || !vel_cmd_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = now();
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
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    end.x = vel_cmd.linear.x;
    end.y = vel_cmd.linear.y;
    end.z = vel_cmd.linear.z;
    
    marker_msg.points = {start, end};
    vel_cmd_marker_pub_->publish(marker_msg);
  }

  void FractionalTeleoperationNode::publishDesiredPositionMarker(
      const Eigen::Vector3d &desired_position)
  {
    if (!enable_marker_visualization_ || !desired_position_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = now();
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

  void FractionalTeleoperationNode::controlUpdate()
  {
    typedef extender_msgs::msg::TeleopCommand Mode;
    
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
    if (adapt_gain_to_alpha_)
    {
      updateGainK();
    }

    // Apply fractional-order differential law: D^alpha x_d(t) = K u(t)
    // This is implemented by integrating the fractional derivative numerically
    Eigen::Vector3d previous_desired_position_linear = desired_position_linear_;
    Eigen::Vector3d previous_desired_position_angular = desired_position_angular_;

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

    if (enable_marker_visualization_) {
      publishDesiredPositionMarker(desired_position_linear_);
    }

    // Compute Cartesian velocities from desired positions
    Eigen::Vector3d cartesian_linear_velocity = 
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        desired_position_linear_, previous_desired_position_linear, dt_); // Relative motion
    Eigen::Vector3d cartesian_angular_velocity = 
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        desired_position_angular_, previous_desired_position_angular, dt_);

    fractional_teleoperation::core::transformApplyModeAndScale(
        cartesian_linear_velocity,
        cartesian_angular_velocity,
        input_frame_,
        current_orientation_,
        mode_,
        Mode::TRANSLATION,
        Mode::ROTATION,
        velocity_scale_);

    // Create and publish velocity command
    geometry_msgs::msg::Twist vel_cmd;
    vel_cmd.linear.x = cartesian_linear_velocity.x();
    vel_cmd.linear.y = cartesian_linear_velocity.y();
    vel_cmd.linear.z = cartesian_linear_velocity.z();
    vel_cmd.angular.x = cartesian_angular_velocity.x();
    vel_cmd.angular.y = cartesian_angular_velocity.y();
    vel_cmd.angular.z = cartesian_angular_velocity.z();

    vel_cmd_pub_->publish(vel_cmd);

    // Publish marker visualization for velocity command
    if (enable_marker_visualization_) {
      publishMarker(vel_cmd);
    }
  }

} // namespace fractional_teleoperation_node

// Main entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fractional_teleoperation_node::FractionalTeleoperationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
