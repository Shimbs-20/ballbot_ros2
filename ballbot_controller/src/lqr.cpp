#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class BallbotLQRNode : public rclcpp::Node {
public:
    BallbotLQRNode() : Node("ballbot_lqr_node") {
        
        // --- Declare ROS 2 Parameters for Nav2/MPPI Tuning ---
        this->declare_parameter("max_motor_torque", 5.0);
        this->declare_parameter("max_yaw_rate", 1.0);
        this->declare_parameter("max_integral", 0.5);
        this->declare_parameter("lpf_alpha", 0.3);

        // Publishers
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribers (cmd_vel receives Nav2 MPPI Twist commands)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BallbotLQRNode::imu_callback, this, std::placeholders::_1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&BallbotLQRNode::joint_states_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BallbotLQRNode::cmd_vel_callback, this, std::placeholders::_1));

        // 100 Hz Control Loop Timer (10 ms)
        timer_ = this->create_wall_timer(10ms, std::bind(&BallbotLQRNode::control_loop_callback, this));
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Ballbot LQR Node (Nav2 Ready) Started!");
    }

private:
    // --- LQR Matrices (Generated from MATLAB LQI) ---
    const double K_LQI[3][10] = {
        {-10.0823,  27.1979,  0.7757,  1.9712,  0.0000, -2.2394,  6.6024,  0.1243,  0.9745,  0.0000},
        {-18.5130, -22.3305,  0.7757, -0.9856,  1.7071, -4.5982, -5.2406,  0.1243, -0.4872,  0.8439},
        { 28.5953,  -4.8674,  0.7757, -0.9856, -1.7071,  6.8376, -1.3619,  0.1243, -0.4872, -0.8439}
    };

    // --- Physical Geometry (From MATLAB/C Snippet) ---
    const double rk = 0.125;  // Ball radius
    const double rw = 0.05;   // Wheel radius

    // --- State Variables ---
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double roll_rate_ = 0.0, pitch_rate_ = 0.0, yaw_rate_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0; 
    double odom_x_ = 0.0, odom_y_ = 0.0; 

    // --- LQR Arrays ---
    double current_state_ [10]= {0};
    double reference_state[10]= {0};
    double error_state_ [10]= {0};
    
    // MPPI / Joystick Targets
    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_yaw_rate_ = 0.0;

    double integral_error_x_ = 0.0;
    double integral_error_y_ = 0.0;
    double prev_filtered_torque[3] = {0.0, 0.0, 0.0};

    rclcpp::Time last_time_;

    // ROS objects
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);

        roll_rate_ = msg->angular_velocity.x;
        pitch_rate_ = msg->angular_velocity.y;
        yaw_rate_ = msg->angular_velocity.z;
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double w1 = 0, w2 = 0, w3 = 0;

        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "wheel1_joint") w1 = msg->velocity[i];
            if (msg->name[i] == "wheel2_joint") w2 = msg->velocity[i];
            if (msg->name[i] == "wheel3_joint") w3 = msg->velocity[i];
        }

        // --- FORWARD KINEMATICS (From user snippet) ---
        // Converts wheel rad/s to ball linear velocity (m/s)
        const double sqrt3 = 1.7320508;
        vx_ = (rw / rk) * (-0.5 * w1 - 0.5 * w2 + 1.0 * w3) / sqrt3;
        vy_ = (rw / rk) * ( 0.866025 * w1 - 0.866025 * w2) / sqrt3;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();

        // Nav2 MPPI outputs linear X and Y for holonomic robots
        target_vx_ = msg->linear.x;
        target_vy_ = msg->linear.y;
        
        target_yaw_rate_ = std::clamp(msg->angular.z, -max_yaw_rate, max_yaw_rate);
    }

    void control_loop_callback() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0) return;

        // Fetch tuning parameters
        double max_integral = this->get_parameter("max_integral").as_double();
        double max_torque = this->get_parameter("max_motor_torque").as_double();
        double lpf_alpha = this->get_parameter("lpf_alpha").as_double();

        // 1. Calculate Odometry (For Nav2 SLAM/Mapping)
        double delta_x = (vx_ * cos(yaw_) - vy_ * sin(yaw_)) * dt;
        double delta_y = (vx_ * sin(yaw_) + vy_ * cos(yaw_)) * dt;
        odom_x_ += delta_x;
        odom_y_ += delta_y;

        // 2. Populate States (STRICT ORDER MATCHING MATLAB `chi(3:10)`)
        // [phi, theta, psi, dIx, dIy, dphi, dtheta, dpsi]
        current_state_[0] = roll_;          // phi
        current_state_[1] = pitch_;         // theta
        current_state_[2] = yaw_;           // psi
        current_state_[3] = vx_;            // dIx
        current_state_[4] = vy_;            // dIy
        current_state_[5] = roll_rate_;     // dphi
        current_state_[6] = pitch_rate_;    // dtheta
        current_state_[7] = yaw_rate_;      // dpsi

        // Set references (We only want to change speeds/yaw)
        reference_state[3] = target_vx_;
        reference_state[4] = target_vy_;
        reference_state[7] = target_yaw_rate_;

        // 3. Compute Base Errors
        for (int i = 0; i < 8; i++) {
            error_state_[i] = current_state_[i] - reference_state[i];
        }

        // 4. Update Integrals (with anti-windup)
        integral_error_x_ += error_state_[3] * dt;
        integral_error_y_ += error_state_[4] * dt;

        integral_error_x_ = std::clamp(integral_error_x_, -max_integral, max_integral);
        integral_error_y_ = std::clamp(integral_error_y_, -max_integral, max_integral);

        // Assign to LQI integral states
        error_state_[3] = integral_error_x_;
        error_state_[4] = integral_error_y_;

        // 5. Matrix Math: Torques = -K * Error
        double physical_torque[3] = {0.0, 0.0, 0.0};
        double max_req_torque = 0.0;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                physical_torque[i] -= K_LQI[i][j] * error_state_[j];
            }
            if (std::abs(physical_torque[i]) > max_req_torque) {
                max_req_torque = std::abs(physical_torque[i]);
            }
        }

        // 6. Proportional Torque Scaling
        if (max_req_torque > max_torque) {
            double scale = max_torque / max_req_torque;
            for (int i = 0; i < 3; i++) physical_torque[i] *= scale;
        }

        // 7. Low-Pass Filter & Publish
        std_msgs::msg::Float64MultiArray torque_msg;
        for (int i = 0; i < 3; i++) {
            double smoothed = (lpf_alpha * physical_torque[i]) + ((1.0 - lpf_alpha) * prev_filtered_torque[i]);
            prev_filtered_torque[i] = smoothed;
            torque_msg.data.push_back(smoothed);
        }
        torque_pub_->publish(torque_msg);

        // 8. Publish Odom and TF
        publish_odometry(current_time);
    }

    void publish_odometry(rclcpp::Time current_time) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = odom_x_;
        t.transform.translation.y = odom_y_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.orientation = t.transform.rotation;
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = yaw_rate_;
        odom_pub_->publish(odom);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallbotLQRNode>());
    rclcpp::shutdown();
    return 0;
}