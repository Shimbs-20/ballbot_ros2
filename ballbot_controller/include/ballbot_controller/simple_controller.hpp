#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Simplecontroller : public rclcpp::Node {
public:
  Simplecontroller(const std::string &name);



private:
  void callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  double wheel_radius_;
  double wheel_base_;
  Eigen::Matrix2d pose_;
};

#endif 
