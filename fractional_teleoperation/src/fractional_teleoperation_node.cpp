#include "fractional_teleoperation/fractional_teleoperation_node.hpp"
#include "fractional_teleoperation/fractional_teleoperation_core.hpp"

#include <algorithm>
#include <cctype>

namespace fractional_teleoperation_node
{
  FractionalTeleoperationNode::FractionalTeleoperationNode(
      const rclcpp::NodeOptions & options)
      : rclcpp::Node("fractional_teleoperation_node", options),
        latest_joystick_(),
        fractional_offset_linear_(Eigen::Vector3d::Zero()),
        fractional_offset_angular_(Eigen::Vector3d::Zero()),
        reference_position_linear_(Eigen::Vector3d::Zero()),
        reference_position_angular_(Eigen::Vector3d::Zero()),
        current_orientation_(Eigen::Quaterniond::Identity()),
        current_alpha_(1.5),
        joystick_was_active_(false),
        joystick_active_duration_(0.0),
        last_linear_scale_(0.0),
        last_angular_scale_(0.0)
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

    // Validate gain normalization mode
    std::transform(adaptive_gain_mode_.begin(), adaptive_gain_mode_.end(), adaptive_gain_mode_.begin(), ::tolower);
    if (
        adaptive_gain_mode_ != "dt" && adaptive_gain_mode_ != "perceptual" &&
        adaptive_gain_mode_ != "geometric_transition")
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid alpha_gain_normalization_mode '%s'. Use 'dt', 'perceptual', or 'geometric_transition'. Defaulting to 'dt'.",
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

    if (k_0_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid k_0=%.3f. Must be > 0. Using default 1.0.",
                  k_0_);
      k_0_ = 1.0;
    }

    if (k_1_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid k_1=%.3f. Must be > 0. Using default 1.0.",
                  k_1_);
      k_1_ = 1.0;
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
    RCLCPP_INFO(get_logger(), "  fractional_offset_gain: %.6f", gain_K_);
    RCLCPP_INFO(get_logger(), "  memory_length: %d", memory_length_);
    RCLCPP_INFO(get_logger(), "  dt: %.6f s", dt_);
    RCLCPP_INFO(get_logger(), "  output_velocity_scale: %.4f", velocity_scale_);
    RCLCPP_INFO(get_logger(), "  global_linear_velocity_saturation: %.4f", global_linear_velocity_saturation_);
    RCLCPP_INFO(get_logger(), "  global_angular_velocity_saturation: %.4f", global_angular_velocity_saturation_);
    RCLCPP_INFO(get_logger(), "  input_frame: %s", input_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  normalize_gain_for_alpha: %s", adapt_gain_to_alpha_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  reference_drift_joystick_threshold: %.4f", reference_drift_joystick_threshold_);
    RCLCPP_INFO(get_logger(), "  joystick_active_threshold: %.4f", joystick_active_threshold_);
    RCLCPP_INFO(get_logger(), "  snap_reference_on_release: %s", snap_reference_on_release_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  linear_offset_scale_max: %.4f", fractional_offset_linear_scale_max_);
    RCLCPP_INFO(get_logger(), "  angular_offset_scale_max: %.4f", fractional_offset_angular_scale_max_);
    RCLCPP_INFO(get_logger(), "  offset_ramp_time: %.4f s", fractional_offset_scale_ramp_time_);
    RCLCPP_INFO(get_logger(), "  offset_ramp_profile: %s", fractional_offset_scale_ramp_profile_.c_str());
    RCLCPP_INFO(get_logger(), "  use_reference_drift: %s", use_reference_drift_ ? "true" : "false");
    if (use_reference_drift_)
    {
      RCLCPP_INFO(get_logger(), "  reference_first_order_rate: %.4f 1/s", reference_drift_rate_);
      RCLCPP_INFO(get_logger(), "  reference_update_mode: %s", reference_update_mode_.c_str());
      if (reference_update_mode_ == "fractional")
      {
        RCLCPP_INFO(get_logger(), "  reference_fractional_alpha: %.4f", reference_alpha_);
        RCLCPP_INFO(get_logger(), "  reference_fractional_gain: %.4f", reference_fractional_gain_);
      }
    }
    if (adapt_gain_to_alpha_)
    {
      RCLCPP_INFO(get_logger(), "  alpha_gain_normalization_mode: %s", adaptive_gain_mode_.c_str());
      RCLCPP_INFO(get_logger(), "  v_max: %.3f m/s", v_max_);
      if (adaptive_gain_mode_ == "perceptual")
      {
        RCLCPP_INFO(get_logger(), "  t_ref: %.3f s", t_ref_);
      }
      else if (adaptive_gain_mode_ == "geometric_transition")
      {
        RCLCPP_INFO(get_logger(), "  k_0: %.6f", k_0_);
        RCLCPP_INFO(get_logger(), "  k_1: %.6f", k_1_);
      }
    }
    
    // Log dynamic alpha parameters if enabled
    if (use_dynamic_alpha_)
    {
      RCLCPP_INFO(get_logger(), "  Dynamic Alpha Enabled:");
      RCLCPP_INFO(get_logger(), "    alpha_min: %.4f", alpha_min_);
      RCLCPP_INFO(get_logger(), "    alpha_max: %.4f", alpha_max_);
      RCLCPP_INFO(get_logger(), "    l_0 (lower threshold): %.4f", l_0_);
      RCLCPP_INFO(get_logger(), "    l_max (upper threshold): %.4f", l_max_);
    }

    // Compute Grünwald-Letnikov coefficients
    gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
      memory_length_, current_alpha_);
    reference_gl_coefficients_ = fractional_teleoperation::core::computeGrunwaldCoefficients(
      memory_length_, reference_alpha_);

    // Initialize history buffers
    linear_history_.clear();
    angular_history_.clear();
    reference_linear_history_.clear();
    reference_angular_history_.clear();

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
    declare_parameter("fractional_offset_gain", 0.1);
    declare_parameter("memory_length", 100);
    declare_parameter("dt", 0.01);
    declare_parameter("output_velocity_scale", 1.0);
    declare_parameter("global_linear_velocity_saturation", 0.0);
    declare_parameter("global_angular_velocity_saturation", 0.0);
    declare_parameter("input_frame", std::string("base"));
    declare_parameter("normalize_gain_for_alpha", true);
    declare_parameter("alpha_gain_normalization_mode", std::string("dt"));
    declare_parameter("v_max", 1.0);
    declare_parameter("t_ref", 1.0);
    declare_parameter("k_0", 1.0);
    declare_parameter("k_1", 1.0);
    declare_parameter("enable_marker_visualization", true);
    declare_parameter("vel_cmd_marker_topic", std::string("/vel_cmd_marker"));
    declare_parameter("vel_cmd_marker_frame_id", std::string("base_link"));
    declare_parameter("vel_cmd_marker_scale_x", 0.03);
    declare_parameter("vel_cmd_marker_scale_y", 0.07);
    declare_parameter("vel_cmd_marker_scale_z", 0.07);
    declare_parameter("desired_position_marker_topic", std::string("/desired_position_marker"));
    declare_parameter("desired_position_marker_scale", 0.04);
    declare_parameter("reference_position_marker_topic", std::string("/reference_position_marker"));
    declare_parameter("reference_position_marker_scale", 0.04);
    declare_parameter("joystick_linear_marker_topic", std::string("/joystick_linear_marker"));
    declare_parameter("use_dynamic_alpha", false);
    declare_parameter("alpha_min", 0.0);
    declare_parameter("alpha_max", 1.0);
    declare_parameter("l_0", 0.1);
    declare_parameter("l_max", 1.0);
    declare_parameter("use_reference_drift", true);
    declare_parameter("reference_first_order_rate", 0.15);
    declare_parameter("reference_drift_joystick_threshold", 0.0);
    declare_parameter("joystick_active_threshold", 0.01);
    declare_parameter("snap_reference_on_release", false);
    declare_parameter("reference_update_mode", std::string("first_order"));
    declare_parameter("reference_fractional_alpha", 0.8);
    declare_parameter("reference_fractional_gain", 0.15);
    declare_parameter("linear_offset_scale_max", 1.0);
    declare_parameter("angular_offset_scale_max", 1.0);
    declare_parameter("offset_ramp_time", 0.5);
    declare_parameter("offset_ramp_profile", std::string("sigmoid"));
    declare_parameter("teleop_cmd_input_topic", std::string("/teleop_cmd"));
    declare_parameter("vel_cmd_topic", std::string("/vel_cmd"));

    alpha_ = get_parameter("alpha").as_double();
    gain_K_ = get_parameter("fractional_offset_gain").as_double();
    memory_length_ = get_parameter("memory_length").as_int();
    dt_ = get_parameter("dt").as_double();
    velocity_scale_ = get_parameter("output_velocity_scale").as_double();
    global_linear_velocity_saturation_ = get_parameter("global_linear_velocity_saturation").as_double();
    global_angular_velocity_saturation_ = get_parameter("global_angular_velocity_saturation").as_double();
    input_frame_ = get_parameter("input_frame").as_string();
    adapt_gain_to_alpha_ = get_parameter("normalize_gain_for_alpha").as_bool();
    adaptive_gain_mode_ = get_parameter("alpha_gain_normalization_mode").as_string();
    v_max_ = get_parameter("v_max").as_double();
    t_ref_ = get_parameter("t_ref").as_double();
    k_0_ = get_parameter("k_0").as_double();
    k_1_ = get_parameter("k_1").as_double();
    enable_marker_visualization_ = get_parameter("enable_marker_visualization").as_bool();
    marker_topic_ = get_parameter("vel_cmd_marker_topic").as_string();
    marker_frame_id_ = get_parameter("vel_cmd_marker_frame_id").as_string();
    marker_scale_x_ = get_parameter("vel_cmd_marker_scale_x").as_double();
    marker_scale_y_ = get_parameter("vel_cmd_marker_scale_y").as_double();
    marker_scale_z_ = get_parameter("vel_cmd_marker_scale_z").as_double();
    desired_position_marker_topic_ = get_parameter("desired_position_marker_topic").as_string();
    desired_position_marker_scale_ = get_parameter("desired_position_marker_scale").as_double();
    reference_position_marker_topic_ = get_parameter("reference_position_marker_topic").as_string();
    reference_position_marker_scale_ = get_parameter("reference_position_marker_scale").as_double();
    joystick_linear_marker_topic_ = get_parameter("joystick_linear_marker_topic").as_string();
    use_dynamic_alpha_ = get_parameter("use_dynamic_alpha").as_bool();
    alpha_min_ = get_parameter("alpha_min").as_double();
    alpha_max_ = get_parameter("alpha_max").as_double();
    l_0_ = get_parameter("l_0").as_double();
    l_max_ = get_parameter("l_max").as_double();
    use_reference_drift_ = get_parameter("use_reference_drift").as_bool();
    reference_drift_rate_ = get_parameter("reference_first_order_rate").as_double();
    reference_drift_joystick_threshold_ = get_parameter("reference_drift_joystick_threshold").as_double();
    joystick_active_threshold_ = get_parameter("joystick_active_threshold").as_double();
    snap_reference_on_release_ = get_parameter("snap_reference_on_release").as_bool();
    reference_update_mode_ = get_parameter("reference_update_mode").as_string();
    reference_alpha_ = get_parameter("reference_fractional_alpha").as_double();
    reference_fractional_gain_ = get_parameter("reference_fractional_gain").as_double();
    fractional_offset_linear_scale_max_ = get_parameter("linear_offset_scale_max").as_double();
    fractional_offset_angular_scale_max_ = get_parameter("angular_offset_scale_max").as_double();
    fractional_offset_scale_ramp_time_ = get_parameter("offset_ramp_time").as_double();
    fractional_offset_scale_ramp_profile_ = get_parameter("offset_ramp_profile").as_string();
    teleop_cmd_input_topic_ = get_parameter("teleop_cmd_input_topic").as_string();

    if (reference_drift_rate_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "reference_first_order_rate must be >= 0. Received %.4f, clamping to 0.",
          reference_drift_rate_);
      reference_drift_rate_ = 0.0;
    }

    if (reference_drift_joystick_threshold_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "reference_drift_joystick_threshold must be >= 0. Received %.4f, clamping to 0.",
          reference_drift_joystick_threshold_);
      reference_drift_joystick_threshold_ = 0.0;
    }

    if (joystick_active_threshold_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "joystick_active_threshold must be >= 0. Received %.4f, clamping to 0.",
          joystick_active_threshold_);
      joystick_active_threshold_ = 0.0;
    }

    if (global_linear_velocity_saturation_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "global_linear_velocity_saturation must be >= 0. Received %.4f, clamping to 0.",
          global_linear_velocity_saturation_);
      global_linear_velocity_saturation_ = 0.0;
    }

    if (global_angular_velocity_saturation_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "global_angular_velocity_saturation must be >= 0. Received %.4f, clamping to 0.",
          global_angular_velocity_saturation_);
      global_angular_velocity_saturation_ = 0.0;
    }

    std::transform(
        reference_update_mode_.begin(),
        reference_update_mode_.end(),
        reference_update_mode_.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (reference_update_mode_ != "first_order" && reference_update_mode_ != "fractional")
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid reference_update_mode '%s'. Use 'first_order' or 'fractional'. Defaulting to 'first_order'.",
          reference_update_mode_.c_str());
      reference_update_mode_ = "first_order";
    }

    if (reference_alpha_ <= 0.0 || reference_alpha_ >= 2.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "Invalid reference_fractional_alpha=%.3f. Must be in (0, 2). Clamping to [0.1, 1.9].",
          reference_alpha_);
      reference_alpha_ = std::clamp(reference_alpha_, 0.1, 1.9);
    }

    if (reference_fractional_gain_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "reference_fractional_gain must be >= 0. Received %.4f, clamping to 0.",
          reference_fractional_gain_);
      reference_fractional_gain_ = 0.0;
    }

    if (alpha_min_ < 0.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "alpha_min must be >= 0. Received %.4f, clamping to 0.",
          alpha_min_);
      alpha_min_ = 0.0;
    }

    if (alpha_max_ >= 2.0)
    {
      RCLCPP_WARN(
          get_logger(),
          "alpha_max must be < 2. Received %.4f, clamping to 1.9.",
          alpha_max_);
      alpha_max_ = 1.9;
    }

    if (alpha_min_ > alpha_max_)
    {
      RCLCPP_WARN(
          get_logger(),
          "alpha_min (%.4f) must be <= alpha_max (%.4f). Setting alpha_min=alpha_max.",
          alpha_min_,
          alpha_max_);
      alpha_min_ = alpha_max_;
    }

    if (fractional_offset_linear_scale_max_ < 0.0)
    {
      RCLCPP_WARN(get_logger(),
          "linear_offset_scale_max must be >= 0. Received %.4f, clamping to 0.",
          fractional_offset_linear_scale_max_);
      fractional_offset_linear_scale_max_ = 0.0;
    }

    if (fractional_offset_angular_scale_max_ < 0.0)
    {
      RCLCPP_WARN(get_logger(),
          "angular_offset_scale_max must be >= 0. Received %.4f, clamping to 0.",
          fractional_offset_angular_scale_max_);
      fractional_offset_angular_scale_max_ = 0.0;
    }

    if (fractional_offset_scale_ramp_time_ < 0.0)
    {
      RCLCPP_WARN(get_logger(),
          "offset_ramp_time must be >= 0. Received %.4f, clamping to 0.",
          fractional_offset_scale_ramp_time_);
      fractional_offset_scale_ramp_time_ = 0.0;
    }

    // Validate ramp profile
    try {
      fractional_offset_scale_ramp_profile_ = fractional_teleoperation::ramp::validateProfileName(fractional_offset_scale_ramp_profile_);
    } catch (const std::invalid_argument& e) {
      RCLCPP_WARN(get_logger(),
          "Invalid ramp profile '%s'. %s. Using 'sigmoid'.",
          fractional_offset_scale_ramp_profile_.c_str(), e.what());
      fractional_offset_scale_ramp_profile_ = "sigmoid";
    }

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
        teleop_cmd_input_topic_, 10,
        std::bind(&FractionalTeleoperationNode::joystickCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribing teleop input on topic: %s", teleop_cmd_input_topic_.c_str());
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

        reference_position_marker_pub_ =
          create_publisher<visualization_msgs::msg::Marker>(reference_position_marker_topic_, 10);
        RCLCPP_INFO(get_logger(), "Publishing reference position marker on topic: %s",
              reference_position_marker_topic_.c_str());

      joystick_linear_marker_pub_ =
          create_publisher<visualization_msgs::msg::Marker>(joystick_linear_marker_topic_, 10);
      RCLCPP_INFO(get_logger(), "Publishing joystick linear marker on topic: %s",
                  joystick_linear_marker_topic_.c_str());
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
      current_alpha_, v_max_, t_ref_, dt_, k_0_, k_1_, adaptive_gain_mode_);
  }

  void FractionalTeleoperationNode::joystickCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    // Update the stored joystick command with the latest message
    latest_joystick_ = msg->twist;
    mode_ = msg->mode;
  }

  void FractionalTeleoperationNode::publishMarker(
      const geometry_msgs::msg::Twist &vel_cmd,
      const Eigen::Vector3d &desired_position)
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
    start.x = desired_position.x();
    start.y = desired_position.y();
    start.z = desired_position.z();
    end.x = start.x + vel_cmd.linear.x;
    end.y = start.y + vel_cmd.linear.y;
    end.z = start.z + vel_cmd.linear.z;
    
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

  void FractionalTeleoperationNode::publishReferencePositionMarker(
      const Eigen::Vector3d &reference_position)
  {
    if (!enable_marker_visualization_ || !reference_position_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = now();
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

  void FractionalTeleoperationNode::publishJoystickLinearMarker(
      const Eigen::Vector3d &joystick_linear,
      const Eigen::Vector3d &reference_position)
  {
    if (!enable_marker_visualization_ || !joystick_linear_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.stamp = now();
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

    // Snapshot reference position for reference velocity computation.
    const Eigen::Vector3d previous_reference_position_linear = reference_position_linear_;
    const Eigen::Vector3d previous_reference_position_angular = reference_position_angular_;

    const double alpha_threshold = 0.001;
    fractional_teleoperation::core::updateDynamicAlphaAndCoefficients(
        use_dynamic_alpha_,
        joystick_linear,
        l_0_,
        l_max_,
      alpha_min_,
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

    // Apply fractional-order differential law on offset:
    // D^alpha Delta x(t) = K u(t)
    // Snapshot previous tick's scaled offsets for velocity: d(scale*Δx)/dt
    Eigen::Vector3d previous_scaled_offset_linear  = fractional_offset_linear_  * last_linear_scale_;
    Eigen::Vector3d previous_scaled_offset_angular = fractional_offset_angular_ * last_angular_scale_;
    const Eigen::Vector3d previous_desired_position_linear =
      previous_reference_position_linear + previous_scaled_offset_linear;
    const Eigen::Vector3d previous_desired_position_angular =
      previous_reference_position_angular + previous_scaled_offset_angular;
    // Snapshot pre-update unscaled offsets for snap-on-release
    const Eigen::Vector3d pre_update_offset_linear  = fractional_offset_linear_;
    const Eigen::Vector3d pre_update_offset_angular = fractional_offset_angular_;

    fractional_offset_linear_ = fractional_teleoperation::core::applyFractionalIntegration(
      joystick_linear,
      linear_history_,
      gl_coefficients_,
      memory_length_,
      dt_,
      current_alpha_,
      gain_K_);
    fractional_offset_angular_ = fractional_teleoperation::core::applyFractionalIntegration(
      joystick_angular,
      angular_history_,
      gl_coefficients_,
      memory_length_,
      dt_,
      current_alpha_,
      gain_K_);

    // Ramp scale: configurable profile from 0 when joystick is inactive to scale_max over ramp_time seconds.
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
    const double ramp_factor = fractional_teleoperation::ramp::computeRampFactor(t_ratio, fractional_offset_scale_ramp_profile_);
    const double current_linear_scale  = fractional_offset_linear_scale_max_  * ramp_factor;
    const double current_angular_scale = fractional_offset_angular_scale_max_ * ramp_factor;

    Eigen::Vector3d scaled_offset_linear  = fractional_offset_linear_  * current_linear_scale;
    Eigen::Vector3d scaled_offset_angular = fractional_offset_angular_ * current_angular_scale;

    // Reconstruct desired position from drifting reference and scaled fractional offset
    Eigen::Vector3d desired_position_linear  = reference_position_linear_  + scaled_offset_linear;
    Eigen::Vector3d desired_position_angular = reference_position_angular_ + scaled_offset_angular;

    // Snap reference to current desired position on joystick release (non-zero -> zero transition)
    if (snap_reference_on_release_ && joystick_was_active_ && !joystick_active)
    {
      reference_position_linear_  = reference_position_linear_  + pre_update_offset_linear  * last_linear_scale_;
      reference_position_angular_ = reference_position_angular_ + pre_update_offset_angular * last_angular_scale_;
      fractional_offset_linear_.setZero();
      fractional_offset_angular_.setZero();
      linear_history_.clear();
      angular_history_.clear();
      previous_scaled_offset_linear.setZero();
      previous_scaled_offset_angular.setZero();
    }
    joystick_was_active_ = joystick_active;
    last_linear_scale_  = current_linear_scale;
    last_angular_scale_ = current_angular_scale;

    Eigen::Vector3d reference_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_angular_velocity = Eigen::Vector3d::Zero();
    if (use_reference_drift_)
    {
      reference_linear_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        reference_position_linear_, previous_reference_position_linear, dt_);
      reference_angular_velocity = fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        reference_position_angular_, previous_reference_position_angular, dt_);
    }

    // Compute Cartesian velocities from scaled fractional offset: d(scale*Δx)/dt
    Eigen::Vector3d cartesian_linear_velocity =
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        scaled_offset_linear, previous_scaled_offset_linear, dt_);
    Eigen::Vector3d cartesian_angular_velocity =
      fractional_teleoperation::core::computeVelocityFromDesiredPosition(
        scaled_offset_angular, previous_scaled_offset_angular, dt_);

    // Compensate for reference drift in the commanded Cartesian velocity.
    // This keeps output speed behavior comparable when drift is enabled.
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
        RCLCPP_DEBUG(get_logger(), "Linear velocity saturated: %.4f -> %.4f", linear_velocity_norm, global_linear_velocity_saturation_);
        const double linear_saturation_scale = global_linear_velocity_saturation_ / linear_velocity_norm;
        // Scale down the entire linear velocity vector to maintain direction while enforcing saturation
        cartesian_linear_velocity *= linear_saturation_scale;

        // Keep internal position states consistent with the saturated commanded velocity.
        desired_position_linear = previous_desired_position_linear +
          linear_saturation_scale * (desired_position_linear - previous_desired_position_linear);
        // reference_position_linear_ = previous_reference_position_linear +
        //   linear_saturation_scale * (reference_position_linear_ - previous_reference_position_linear);

        // Reconcile scaled and unscaled offsets after saturation-induced state correction.
        scaled_offset_linear = desired_position_linear - reference_position_linear_;
        if (current_linear_scale > 1e-9)
        {
          fractional_offset_linear_ = scaled_offset_linear / current_linear_scale;
        }
        else
        {
          fractional_offset_linear_.setZero();
        }
      }
    }

    if (global_angular_velocity_saturation_ > 0.0)
    {
      const double angular_velocity_norm = cartesian_angular_velocity.norm();
      if (angular_velocity_norm > global_angular_velocity_saturation_)
      {
        RCLCPP_DEBUG(get_logger(), "Angular velocity saturated: %.4f -> %.4f", angular_velocity_norm, global_angular_velocity_saturation_);
        const double angular_saturation_scale = global_angular_velocity_saturation_ / angular_velocity_norm;
        // Scale down the entire angular velocity vector to maintain direction while enforcing saturation
        cartesian_angular_velocity *= angular_saturation_scale;

        // Keep internal position states consistent with the saturated commanded velocity.
        desired_position_angular = previous_desired_position_angular +
          angular_saturation_scale * (desired_position_angular - previous_desired_position_angular);
        reference_position_angular_ = previous_reference_position_angular +
          angular_saturation_scale * (reference_position_angular_ - previous_reference_position_angular);

        // Reconcile scaled and unscaled offsets after saturation-induced state correction.
        scaled_offset_angular = desired_position_angular - reference_position_angular_;
        if (current_angular_scale > 1e-9)
        {
          fractional_offset_angular_ = scaled_offset_angular / current_angular_scale;
        }
        else
        {
          fractional_offset_angular_.setZero();
        }
      }
    }

    if (use_reference_drift_ &&
        joystick_linear.norm() >= reference_drift_joystick_threshold_)
    {
      if (reference_update_mode_ == "fractional")
      {
      reference_position_linear_ = fractional_teleoperation::core::updateReferencePositionFractional(
        reference_position_linear_,
        desired_position_linear,
        reference_linear_history_,
        reference_gl_coefficients_,
        memory_length_,
        dt_,
        reference_alpha_,
        reference_fractional_gain_);
      reference_position_angular_ = fractional_teleoperation::core::updateReferencePositionFractional(
        reference_position_angular_,
        desired_position_angular,
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
        reference_position_linear_, desired_position_linear, reference_drift_rate_, dt_);
      reference_position_angular_ = fractional_teleoperation::core::updateReferencePosition(
        reference_position_angular_, desired_position_angular, reference_drift_rate_, dt_);
      }
    }

    // Create and publish velocity command
    geometry_msgs::msg::Twist vel_cmd;
    vel_cmd.linear.x = cartesian_linear_velocity.x();
    vel_cmd.linear.y = cartesian_linear_velocity.y();
    vel_cmd.linear.z = cartesian_linear_velocity.z();
    vel_cmd.angular.x = cartesian_angular_velocity.x();
    vel_cmd.angular.y = cartesian_angular_velocity.y();
    vel_cmd.angular.z = cartesian_angular_velocity.z();


    if (enable_marker_visualization_) {
      publishDesiredPositionMarker(desired_position_linear);
      publishReferencePositionMarker(reference_position_linear_);
      publishJoystickLinearMarker(joystick_linear, reference_position_linear_);
    }

    vel_cmd_pub_->publish(vel_cmd);

    // Publish marker visualization for velocity command
    if (enable_marker_visualization_) {
      publishMarker(vel_cmd, desired_position_linear);
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
