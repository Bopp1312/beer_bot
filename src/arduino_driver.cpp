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
    serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << "\n";
        return;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << "\n";
        close(serial_port);
        return;
    }

    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD; // 115200 baud, 8 data bits, no parity, 1 stop bit
    tty.c_iflag = IGNPAR;                         // Ignore framing and parity errors
    tty.c_oflag = 0;                              // Raw output
    tty.c_lflag = 0;                              // Non-canonical input

    if (tcflush(serial_port, TCIFLUSH) != 0) {
        std::cerr << "Error flushing serial port: " << strerror(errno) << "\n";
        close(serial_port);
        return;
    }

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << "\n";
        close(serial_port);
        return;
    }

    std::cout << "Serial port initialized successfully.\n";
  }

  ~ArduinoDriverNode()
  {
    if(serial_port >= 0)
    {
      std::string command = construct_motor_command(0, 0);
      write(serial_port, command.c_str(), command.size());
      close(serial_port);
      RCLCPP_INFO(this->get_logger(),"Serial port closed");
    }
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (serial_port <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open!");
      return;
    }

    double linear_velocity = msg->linear.x;   // Linear Velocity (m/s)
    double angular_velocity = msg->angular.z; // Angular Velocity (rad/s)

    double wheel_radius = 32.0/1000.0; // meters
    double qpp_rev = 1500/M_PI; // qpp per revolution

    double qpps_mps = 400.0; // qpps/m/s convert qpps to meters per second
    // Convert velocities into left and right motor speeds
    const double wheel_base = 0.150; // Distance between wheels (m)
    double left_speed = linear_velocity - (angular_velocity * wheel_base / 2.0);
    double right_speed = linear_velocity + (angular_velocity * wheel_base / 2.0);

    int32_t left_motor_speed = static_cast<int32_t>(left_speed / wheel_radius * qpp_rev);
    int32_t right_motor_speed = static_cast<int32_t>(right_speed / wheel_radius * qpp_rev * -1);

    // Construct the command string for the motor controller
    std::string command = construct_motor_command(left_motor_speed, right_motor_speed);

    // Send the command over the serial port
    write(serial_port, command.c_str(), command.size());
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());

  }

  std::string construct_motor_command(int32_t left_speed, int32_t right_speed) 
  {
    return std::to_string(left_speed) + "," + std::to_string(right_speed) + "\n";
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
