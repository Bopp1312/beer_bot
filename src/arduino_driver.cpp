#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>  // Include the serial library

class SerialTwistNode : public rclcpp::Node {
public:
  SerialTwistNode() : Node("serial_twist_node") {
    // Create a subscription to the /cmd_vel topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&SerialTwistNode::twist_callback, this, std::placeholders::_1));

    // Initialize the serial connection
    try {
      serial_port_.setPort("/dev/ttyUSB0"); // Replace with your serial port
      serial_port_.setBaudrate(38400);     // Replace with your motor controller baud rate
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial_port_.setTimeout(timeout);
      serial_port_.open();
    } catch (const serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
    }

    if (serial_port_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open.");
    }
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!serial_port_.isOpen()) {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open!");
      return;
    }

    // Extract linear and angular velocity from the Twist message
    double linear_velocity = msg->linear.x;   // Forward/backward speed
    double angular_velocity = msg->angular.z; // Rotational speed

    // Convert velocities into left and right motor speeds
    const double wheel_base = 0.5; // Distance between wheels (meters)
    double left_speed = linear_velocity - (angular_velocity * wheel_base / 2.0);
    double right_speed = linear_velocity + (angular_velocity * wheel_base / 2.0);

    // Convert to motor-specific units (e.g., scale to a range like 0-255 or unsigned 32-bit integers)
    int32_t left_motor_speed = static_cast<int32_t>(left_speed * 1000);  // Example scaling
    int32_t right_motor_speed = static_cast<int32_t>(right_speed * 1000); // Example scaling

    // Construct the command string for the motor controller
    std::string command = construct_motor_command(left_motor_speed, right_motor_speed);

    // Send the command over the serial port
    try {
      serial_port_.write(command);
      RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
    } catch (const serial::SerialException &e) {
      RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
    }
  }

  std::string construct_motor_command(int32_t left_speed, int32_t right_speed) {
    // Example: Create a comma-separated command string "L12345,R67890\n"
    return "L" + std::to_string(left_speed) + ",R" + std::to_string(right_speed) + "\n";
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  serial::Serial serial_port_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTwistNode>());
  rclcpp::shutdown();
  return 0;
}
