
// BUG FIX 1: Removed hardcoded absolute include path.
// Use the package-relative include path that CMake/colcon sets up.
#include "ballbot_controller/simple_controller.hpp"
 
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
 
Simplecontroller::Simplecontroller(const std::string &name) : Node(name) {
 
  // BUG FIX 2: Parameter names must match what controller_launch.py passes:
  //   "wheel_radius"    → already correct
  //   "wheel_base"      → WRONG; launch sends "wheel_separation", not "wheel_base"
  // Default values also corrected to match URDF/YAML (radius=0.17, sep=0.33).
  wheel_radius_ = declare_parameter("wheel_radius", 0.17);
  wheel_base_   = declare_parameter("wheel_separation", 0.33);  // was "wheel_base"
 
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius "     << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_base_);
 
  // BUG FIX 3: Subscriber topic was "/ballbot/cmd_vel" (non-existent).
  // The correct topic is "/ballbot_controller/cmd_vel" — this is what
  // twist_relay publishes (TwistStamped) for the simple_controller path.
  sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/ballbot_controller/cmd_vel", 10,
      std::bind(&Simplecontroller::callback, this, std::placeholders::_1));
 
  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/simple_velocity_controller/commands", 10);
 
  // Kinematic conversion matrix  [v_left, v_right]^T = M^{-1} * [v_lin, omega]^T
  pose_ << wheel_radius_ / 2,  wheel_radius_ / 2,
           wheel_radius_ / wheel_base_, -wheel_radius_ / wheel_base_;
 
  RCLCPP_INFO_STREAM(get_logger(), "Conversion matrix:\n" << pose_);
}
 
void Simplecontroller::callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  std_msgs::msg::Float64MultiArray wheel_speed_msg;
 
  Eigen::Vector2d vel(msg->twist.linear.x, msg->twist.angular.z);
  Eigen::Vector2d wheel_vel = pose_.inverse() * vel;
 
  wheel_speed_msg.data.push_back(wheel_vel[0]);
  wheel_speed_msg.data.push_back(wheel_vel[1]);
 
  pub_->publish(wheel_speed_msg);
}
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Simplecontroller>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
 