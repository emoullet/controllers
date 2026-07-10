#pragma once

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "extender_msgs/msg/teleop_command.hpp"
#include "fractional_teleoperation/fractional_teleoperation_core.hpp"
#include "fractional_teleoperation/ramp_profile.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"
#include "signal_processing/one_euro_filter.hpp"

namespace fractional_teleoperation_controller
{
  /// @brief A ROS2 controller implementing fractional-order joystick teleoperation
  /// for robotic arm in Cartesian space using Grünwald-Letnikov discrete approximation.
  /// Desired motion follows: D^alpha x_d(t) = K u(t)
  class FractionalTeleoperationController : public controller_interface::ControllerInterface
  {
  public:
    FractionalTeleoperationController();
    virtual ~FractionalTeleoperationController();

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

  private:
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      auto node = get_node();
      if (!node->has_parameter(name))
      {
        node->declare_parameter(name, default_value);
      }
      variable = node->get_parameter(name).get_value<T>();
    }

    /**
     * @brief Get parameter from preferred name, fallback to a legacy alias if present.
     * @tparam T Type of the parameter.
     * @param preferred_name Preferred parameter name.
     * @param legacy_name Legacy parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if neither parameter is set.
     */
    template <typename T>
    void declare_and_get_parameter_alias(
      const std::string &preferred_name,
      const std::string &legacy_name,
      T &variable,
      const T &default_value)
    {
      auto node = get_node();
      if (node->has_parameter(preferred_name))
      {
        variable = node->get_parameter(preferred_name).get_value<T>();
        return;
      }

      if (node->has_parameter(legacy_name))
      {
        variable = node->get_parameter(legacy_name).get_value<T>();
        return;
      }

      node->declare_parameter(preferred_name, default_value);
      variable = node->get_parameter(preferred_name).get_value<T>();
    }

    void loadParameters();

    void declareParameters();

    void validateAndNormalizeParameters();

    void resetControllerState();

    void setupSubscribers();

    void declarePublishers();

    void activatePublishers();

    void deactivatePublishers();

    bool setupRobotInterface();

    void updateGainK();

    void readJoystickInput(
      Eigen::Vector3d &joystick_linear,
      Eigen::Vector3d &joystick_angular) const;

    Eigen::Vector3d applyDeadbandToVector(const Eigen::Vector3d &input, double deadband) const;

    bool isJoystickInputFresh() const;

    void integrateDesiredOffsets(
      const Eigen::Vector3d &joystick_linear,
      const Eigen::Vector3d &joystick_angular);

    void updateJoystickActivityDuration(bool joystick_active);

    void applySnapOnRelease(
      bool joystick_active,
      const Eigen::Vector3d &pre_update_offset_linear,
      const Eigen::Vector3d &pre_update_offset_angular,
      Eigen::Vector3d &previous_scaled_offset_linear,
      Eigen::Vector3d &previous_scaled_offset_angular);

    void applyLinearVelocitySaturation(
      Eigen::Vector3d &cartesian_linear_velocity,
      const Eigen::Vector3d &previous_desired_position_linear,
      Eigen::Vector3d &desired_linear,
      double current_linear_scale,
      Eigen::Vector3d &scaled_offset_linear);

    void applyAngularVelocitySaturation(
      Eigen::Vector3d &cartesian_angular_velocity,
      const Eigen::Vector3d &previous_desired_position_angular,
      const Eigen::Vector3d &previous_reference_position_angular,
      Eigen::Vector3d &desired_angular,
      double current_angular_scale,
      Eigen::Vector3d &scaled_offset_angular);

    void computeReferenceDriftVelocity(
      const Eigen::Vector3d &previous_reference_position_linear,
      const Eigen::Vector3d &previous_reference_position_angular,
      Eigen::Vector3d &reference_linear_velocity,
      Eigen::Vector3d &reference_angular_velocity) const;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> updateReferenceDrift(
      double joystick_linear_norm,
      const Eigen::Vector3d &desired_linear,
      const Eigen::Vector3d &desired_angular);

    robot_interfaces::CartesianVelocity filterVelocityCommand(
      const robot_interfaces::CartesianVelocity &raw_vel_cmd,
      double timestamp_sec);

    controller_interface::return_type publishVelocityCommand(
      const robot_interfaces::CartesianVelocity &latest_vel_cmd);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> rampDesiredOffset(
      const Eigen::Vector3d &desired_offset_linear,
      const Eigen::Vector3d &desired_offset_angular,
      double joystick_active_duration,
      double &current_linear_scale,
      double &current_angular_scale) const;

    /// Callback to receive normalized 3D joystick commands
    void joystickCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);

    /// Publish marker visualization for velocity command
    void publishMarker(
      const robot_interfaces::CartesianVelocity &vel_cmd,
      const Eigen::Vector3d &desired_position);

    void publishDesiredPositionMarker(const Eigen::Vector3d &desired_position);

    void publishReferencePositionMarker(const Eigen::Vector3d &reference_position);

    void publishJoystickLinearMarker(
      const Eigen::Vector3d &joystick_linear,
      const Eigen::Vector3d &reference_position);

    void publishEePoseTranslationMarker(const Eigen::Vector3d &ee_pose_translation);

    /// Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    uint8_t mode_{extender_msgs::msg::TeleopCommand::BOTH};

    /// Stores the latest joystick command received (normalized 3D)
    geometry_msgs::msg::Twist latest_joystick_;

    /// Desired position state (result of fractional integration)
    Eigen::Vector3d desired_offset_linear_;
    Eigen::Vector3d desired_offset_angular_;
    Eigen::Vector3d reference_position_linear_;
    Eigen::Vector3d reference_position_angular_;
    Eigen::Quaterniond orientation_error_;
    

    /// Robot state
    Eigen::Quaterniond current_orientation_;
    Eigen::Quaterniond reference_orientation_;
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_linear_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    
    /// Current alpha values (may be dynamic)
    double current_linear_alpha_;
    double current_angular_alpha_;

    /// Fractional-order parameters
    double linear_alpha_;    // Linear fractional order (0 < alpha < 2)
    double angular_alpha_;   // Angular fractional order (0 < alpha < 2)
    double linear_gain_K_;   // Linear gain K in the fractional law D^alpha x_d = K u
    double angular_gain_K_;  // Angular gain K in the fractional law D^alpha x_d = K u
    int memory_length_;      // Number of past samples to keep for GL approximation
    int update_rate_;     // Update rate (Hz) from controller manager
    double dt_;              // Time step (seconds)
    double velocity_scale_{1.0};

    double global_linear_velocity_saturation_{0.0};
    double global_angular_velocity_saturation_{0.0};

    bool enable_linear_one_euro_filter_{false};
    double linear_one_euro_frequency_{100.0};
    double linear_one_euro_mincutoff_{1.0};
    double linear_one_euro_beta_{0.1};
    double linear_one_euro_dcutoff_{1.0};
    bool enable_angular_one_euro_filter_{false};
    double angular_one_euro_frequency_{100.0};
    double angular_one_euro_mincutoff_{1.0};
    double angular_one_euro_beta_{0.1};
    double angular_one_euro_dcutoff_{1.0};
    signal_processing::OneEuroFilter3d linear_velocity_filter_;
    signal_processing::OneEuroFilter3d angular_velocity_filter_;

    bool linear_normalize_gain_for_alpha_{true};
    bool angular_normalize_gain_for_alpha_{true};
    std::string linear_alpha_gain_normalization_mode_{"dt"};
    std::string angular_alpha_gain_normalization_mode_{"dt"};
    double linear_v_max_{1.0};
    double angular_v_max_{1.0};
    double linear_t_ref_{1.0};
    double angular_t_ref_{1.0};
    double linear_k_0_{1.0};
    double angular_k_0_{1.0};
    double linear_k_1_{1.0};
    double angular_k_1_{1.0};

    bool use_reference_drift_{true};
    double reference_drift_rate_{0.15};
    double reference_drift_joystick_threshold_{0.0};
    double joystick_active_threshold_{0.01};
    double joystick_timeout_sec_{0.2};
    rclcpp::Time last_joystick_stamp_{0, 0, RCL_ROS_TIME};
    bool joystick_message_received_{false};
    double linear_deadband_{0.01};
    double angular_deadband_{0.03};
    double k_orient_{4.0};
    double kp_linear_{1.0};
    double kd_linear_{0.1};
    double kp_orient_{1.0};
    double kd_orient_{0.1};
    bool snap_reference_on_release_{false};
    std::string reference_update_mode_{"first_order"};
    double reference_alpha_{0.8};
    double reference_fractional_gain_{0.15};
    double fractional_offset_linear_scale_max_{1.0};
    double fractional_offset_angular_scale_max_{1.0};
    double fractional_offset_scale_ramp_time_{0.5};
    std::string fractional_offset_scale_ramp_profile_{"sigmoid"};

    /// Dynamic alpha parameters
    bool use_dynamic_alpha_linear_{false};
    bool use_dynamic_alpha_angular_{false};
    double linear_alpha_min_{0.0};
    double linear_alpha_max_{1.0};
    double linear_l_0_{0.1};
    double linear_l_max_{1.0};
    double angular_alpha_min_{0.0};
    double angular_alpha_max_{1.0};
    double angular_l_0_{0.1};
    double angular_l_max_{1.0};
    double alpha_threshold_{0.001};
    double last_linear_alpha_{0.0};
    double last_angular_alpha_{0.0};

    /// Grünwald-Letnikov coefficients
    std::vector<double> linear_gl_coefficients_;
    std::vector<double> angular_gl_coefficients_;
    std::vector<double> reference_gl_coefficients_;

    /// History buffers for fractional integration
    std::deque<Eigen::Vector3d> linear_history_;
    std::deque<Eigen::Vector3d> angular_history_;
    std::deque<Eigen::Vector3d> reference_linear_history_;
    std::deque<Eigen::Vector3d> reference_angular_history_;

    bool joystick_was_active_{false};
    double joystick_active_duration_{0.0};
    double last_linear_scale_{0.0};
    double last_angular_scale_{0.0};

    std::string teleop_cmd_input_topic_{"/teleop_cmd"};
    std::string tool_frame_{"tool0"};
    std::string base_frame_{"base_link"};

    /// Subscription for joystick commands
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr joystick_sub_;

    /// Subscription for robot description
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    std::string cached_robot_description_;

    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);

    /// Publisher for velocity command marker visualization
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub_;

    /// Publisher for velocity command marker visualization
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      vel_cmd_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      desired_position_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      reference_position_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      joystick_linear_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      ee_pose_translation_marker_pub_;

    /// Marker visualization parameters
    bool enable_marker_visualization_{true};
    std::string marker_topic_{"/vel_cmd_marker"};
    std::string marker_frame_id_{"base_link"};
    double marker_scale_x_{0.03};
    double marker_scale_y_{0.07};
    double marker_scale_z_{0.07};
    std::string desired_position_marker_topic_{"/desired_position_marker"};
    double desired_position_marker_scale_{0.04};
    std::string reference_position_marker_topic_{"/reference_position_marker"};
    double reference_position_marker_scale_{0.04};
    std::string joystick_linear_marker_topic_{"/joystick_linear_marker"};
    std::string ee_pose_translation_marker_topic_{"/ee_pose_translation_marker"};
    double ee_pose_translation_marker_scale_{0.04};

    // Frame for interpreting incoming joystick commands: "base" or "ee"
    std::string input_frame_{"base"};

    std::vector<std::string> command_names_;
    std::string robot_type_{"franka_velocity"};
    bool use_topic_output_{false};
    std::string vel_cmd_output_topic_{"/fractional_teleoperation_controller/velocity_command"};
  };
} // namespace fractional_teleoperation_controller
