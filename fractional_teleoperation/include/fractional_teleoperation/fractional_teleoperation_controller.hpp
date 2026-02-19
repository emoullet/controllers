#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "extender_msgs/msg/teleop_command.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

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

    void loadParameters();

    void declarePublishers();

    bool setupRobotInterface();

    /// Callback to receive normalized 3D joystick commands
    void joystickCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);

    /// Publish marker visualization for velocity command
    void publishMarker(const robot_interfaces::CartesianVelocity &vel_cmd);

    /// Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    uint8_t mode_{extender_msgs::msg::TeleopCommand::BOTH};

    /// Stores the latest joystick command received (normalized 3D)
    geometry_msgs::msg::Twist latest_joystick_;

    /// Desired position state (result of fractional integration)
    Eigen::Vector3d desired_position_linear_;
    Eigen::Vector3d desired_position_angular_;

    /// Robot state
    Eigen::Quaterniond current_orientation_;
    
    /// Current alpha value (may be dynamic)
    double current_alpha_;

    /// Fractional-order parameters
    double alpha_;           // Fractional order (0 < alpha < 2)
    double gain_K_;          // Gain K in the fractional law D^alpha x_d = K u
    int memory_length_;      // Number of past samples to keep for GL approximation
    double dt_;              // Time step (seconds)

    /// Dynamic alpha parameters
    bool use_dynamic_alpha_{false};  // Whether to enable dynamic alpha based on joystick norm
    double alpha_max_{1.0};          // Maximum alpha value for dynamic adjustment
    double l_0_{0.1};                // Lower threshold for joystick norm
    double l_max_{1.0};              // Upper threshold for joystick norm
    double last_alpha_{0.0};         // Track last alpha to detect changes

    /// Grünwald-Letnikov coefficients
    std::vector<double> gl_coefficients_;

    /// History buffers for fractional integration
    std::deque<Eigen::Vector3d> linear_history_;
    std::deque<Eigen::Vector3d> angular_history_;

    /// Subscription for joystick commands
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr joystick_sub_;

    /// Publisher for velocity command marker visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_cmd_marker_pub_;

    /// Marker visualization parameters
    bool enable_marker_visualization_{true};
    std::string marker_topic_{"/vel_cmd_marker"};
    std::string marker_frame_id_{"base_link"};
    double marker_scale_x_{0.03};
    double marker_scale_y_{0.07};
    double marker_scale_z_{0.07};

    // Frame for interpreting incoming joystick commands: "base" or "ee"
    std::string input_frame_{"base"};

    std::vector<std::string> command_names_;
    std::string robot_type_{"franka_velocity"};

    /// Velocity scaling parameter for output
    double velocity_scale_{1.0};
  };
} // namespace fractional_teleoperation_controller
