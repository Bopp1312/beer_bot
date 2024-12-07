#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class ArduinoDriverNode : public rclcpp::Node {
public:
  ArduinoDriverNode() : Node("arduino_driver_node") {

    // Create a subscription to the /cmd_vel topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ArduinoDriverNode::twist_callback, this, std::placeholders::_1));

    // Initialize the serial connection
   int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
        std::cerr << "Error opening serial port\n";
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting terminal attributes\n";
    }

    tty.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tcflush(serial_port, TCIFLUSH);
    tcsetattr(serial_port, TCSANOW, &tty);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (serial_port <= 0) {
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
  
    write(serial_port, command.c_str(), command.size());
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());

  }

  std::string construct_motor_command(int32_t left_speed, int32_t right_speed) {
    // Example: Create a comma-separated command string "L12345,R67890\n"
    return "L" + std::to_string(left_speed) + ",R" + std::to_string(right_speed) + "\n";
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  int serial_port;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
