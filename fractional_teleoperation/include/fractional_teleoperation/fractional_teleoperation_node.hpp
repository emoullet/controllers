#ifndef FRACTIONAL_TELEOPERATION_NODE_HPP
#define FRACTIONAL_TELEOPERATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "extender_msgs/msg/teleop_command.hpp"

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
    void controlUpdate();

    // Parameters
    double alpha_;
    double gain_K_;
    int memory_length_;
    double dt_;
    double velocity_scale_;
    std::string input_frame_;
    bool adapt_gain_to_alpha_;
    std::string adaptive_gain_mode_;
    double v_max_;
    double t_ref_;
    
    // Dynamic alpha parameters
    bool use_dynamic_alpha_;
    double alpha_max_;
    double l_0_;
    double l_max_;
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

    // Subscribers and Publishers
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr joystick_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_cmd_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr desired_position_marker_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // State
    geometry_msgs::msg::Twist latest_joystick_;
    extender_msgs::msg::TeleopCommand::_mode_type mode_;
    Eigen::Vector3d desired_position_linear_;
    Eigen::Vector3d desired_position_angular_;
    Eigen::Quaterniond current_orientation_;
    double current_alpha_;

    // Grünwald-Letnikov coefficients
    std::vector<double> gl_coefficients_;
    
    // History buffers for fractional integration
    std::deque<Eigen::Vector3d> linear_history_;
    std::deque<Eigen::Vector3d> angular_history_;
  };

} // namespace fractional_teleoperation_node

#endif // FRACTIONAL_TELEOPERATION_NODE_HPP
