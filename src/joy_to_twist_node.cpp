#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToTwistNode : public rclcpp::Node
{
public:
    JoyToTwistNode() : Node("joy_to_twist_node")
    {
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyToTwistNode::joy_callback, this, std::placeholders::_1));
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "JoyToTwistNode started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Create a Twist message and map joystick axes to linear.x and angular.z
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = msg->axes[1];   // Joystick forward/backward axis
        twist_msg.angular.z = msg->axes[0]; // Joystick left/right axis

        // Publish the Twist message
        twist_publisher_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Published Twist: linear.x = %.2f, angular.z = %.2f",twist_msg.linear.x, twist_msg.angular.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwistNode>());
    rclcpp::shutdown();
    return 0;
}