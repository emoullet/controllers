#ifndef FRACTIONAL_TELEOPERATION_NODE_HPP
#define FRACTIONAL_TELEOPERATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "extender_msgs/msg/teleop_command.hpp"
#include "fractional_teleoperation/ramp_profile.hpp"

#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <memory>

namespace fractional_teleoperation_node
{
  class FractionalTeleoperationNode : public rclcpp::Node
  {
  public:
    FractionalTeleoperationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FractionalTeleoperationNode() = default;

  private:
    void loadParameters();
    void declareSubscribers();
    void declarePublishers();
    void joystickCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);

    void updateGainK();
    
    void publishMarker(
      const geometry_msgs::msg::Twist &vel_cmd,
      const Eigen::Vector3d &desired_position);
    void publishDesiredPositionMarker(const Eigen::Vector3d &desired_position);
    void publishReferencePositionMarker(const Eigen::Vector3d &reference_position);
    void publishJoystickLinearMarker(
      const Eigen::Vector3d &joystick_linear,
      const Eigen::Vector3d &reference_position);
    void controlUpdate();

    // Parameters
    double alpha_;
    double gain_K_;
    int memory_length_;
    double dt_;
    double velocity_scale_;
    double global_linear_velocity_saturation_;
    double global_angular_velocity_saturation_;
    std::string input_frame_;
    bool adapt_gain_to_alpha_;
    std::string adaptive_gain_mode_;
    double v_max_;
    double t_ref_;
    double k_0_;
    double k_1_;
    bool use_reference_drift_;
    double reference_drift_rate_;
    double reference_drift_joystick_threshold_;
    double joystick_active_threshold_;
    bool snap_reference_on_release_;
    std::string reference_update_mode_;
    double reference_alpha_;
    double reference_fractional_gain_;
    double fractional_offset_linear_scale_max_;
    double fractional_offset_angular_scale_max_;
    double fractional_offset_scale_ramp_time_;
    std::string fractional_offset_scale_ramp_profile_;
    std::string teleop_cmd_input_topic_;

    // Dynamic alpha parameters
    bool use_dynamic_alpha_;
    double alpha_min_;
    double alpha_max_;
    double l_0_;
    double l_max_;
    double alpha_threshold_;
    double last_alpha_;

    // Marker visualization parameters
    bool enable_marker_visualization_;
    std::string marker_topic_;
    std::string marker_frame_id_;
    double marker_scale_x_;
    double marker_scale_y_;
    double marker_scale_z_;
    std::string desired_position_marker_topic_;
    double desired_position_marker_scale_;
    std::string reference_position_marker_topic_;
    double reference_position_marker_scale_;
    std::string joystick_linear_marker_topic_;

    // Subscribers and Publishers
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr joystick_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_cmd_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr desired_position_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_position_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr joystick_linear_marker_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // State
    geometry_msgs::msg::Twist latest_joystick_;
    extender_msgs::msg::TeleopCommand::_mode_type mode_;
    Eigen::Vector3d fractional_offset_linear_;
    Eigen::Vector3d fractional_offset_angular_;
    Eigen::Vector3d reference_position_linear_;
    Eigen::Vector3d reference_position_angular_;
    Eigen::Quaterniond current_orientation_;
    double current_alpha_;
    bool joystick_was_active_;
    double joystick_active_duration_;
    double last_linear_scale_;
    double last_angular_scale_;

    // Grünwald-Letnikov coefficients
    std::vector<double> gl_coefficients_;
    std::vector<double> reference_gl_coefficients_;
    
    // History buffers for fractional integration
    std::deque<Eigen::Vector3d> linear_history_;
    std::deque<Eigen::Vector3d> angular_history_;
    std::deque<Eigen::Vector3d> reference_linear_history_;
    std::deque<Eigen::Vector3d> reference_angular_history_;
  };

} // namespace fractional_teleoperation_node

#endif // FRACTIONAL_TELEOPERATION_NODE_HPP
