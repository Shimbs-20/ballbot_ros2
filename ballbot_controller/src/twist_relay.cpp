#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

class twist_relay : public rclcpp::Node {
public:
  twist_relay() : Node("twist_relay") {

    // twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
    // this node  → /ballbot_controller/cmd_vel          (TwistStamped)
    // diff_drive_controller reads TwistStamped (use_stamped_vel: true)
    controller_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ballbot_controller/cmd_vel_unstamped", 10,
        std::bind(&twist_relay::controller_twist_callback, this,
                  std::placeholders::_1));

    controller_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ballbot_controller/cmd_vel", 10);

    // BUG FIX 16: Joy path removed entirely from C++ version.
    // joy_sub_ subscribed to /input_joy/cmd_vel_stamped (TwistStamped) and
    // joy_pub_ published Twist on /input_joy/cmd_vel.
    // But joy_teleop already publishes Twist on /input_joy/cmd_vel directly.
    // This caused the "topic contains more than one type" error you saw.
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;

  void controller_twist_callback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = get_clock()->now();
    twist_stamped.header.frame_id = "base_footprint";
    twist_stamped.twist = *msg;
    controller_pub_->publish(twist_stamped);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<twist_relay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}