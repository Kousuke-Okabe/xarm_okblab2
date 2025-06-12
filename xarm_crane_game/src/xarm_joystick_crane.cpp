#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

class JoystickCraneNode : public rclcpp::Node{
    public:
        JoystickCraneNode() : Node("joystick_crane_node") {
            servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "delta_twist_cmds", 10
            );
        
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&JoystickCraneNode::subscribe_joy, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(
                100ms, std::bind(&JoystickCraneNode::timer_callback, this)
            );

            this->declare_parameter<std::double_t>("velocity_gain", 1.0);

            timer_param = this->create_wall_timer(
                10s, std::bind(&JoystickCraneNode::timer_param_callback, this)
            );
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        auto geometry_msgs::msg::TwistStamped twist_cmd_;
        double Kp;

        void subscribe_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
            twist_cmd_.header.stamp = this->now();
            twist_cmd_.header.frame_id = "base_link";

            // Assuming axes 0, 1, 2 are for x, y, z movements
            twist_cmd_.twist.linear.x = msg->axes[0];
            twist_cmd_.twist.linear.y = msg->axes[1];
            twist_cmd_.twist.linear.z = msg->axes[2];

            // Assuming axes 3, 4, 5 are for angular movements
            twist_cmd_.twist.angular.x = msg->axes[3];
            twist_cmd_.twist.angular.y = msg->axes[4];
            twist_cmd_.twist.angular.z = msg->axes[5];
        }

        void timer_callback() {
            servo_pub_->publish(twist_cmd_);
        }

        void timer_param_callback() {
            Kp = this->get_parameter("velocity_gain").as_double();
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickCraneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}