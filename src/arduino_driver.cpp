#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct Position
{
  double x = 0;
  double y = 0;
  double theta = 0;
};

struct Wheel_Velocity
{
  double x = 0;
  double y = 0;
};

struct Robot_Velocity
{
  double omega = 0;
  double linear = 0;
};

struct Encoder
{
  int32_t left = 0;
  int32_t right = 0;
  
  Encoder operator-(const Encoder &other) const
  { 
    Encoder result;
    result.left = left - other.left;
    result.right = right - other.right;
    return result;
  }

  Encoder operator+(const Encoder &other) const
  {
    Encoder result;
    result.left = left + other.left;
    result.right = right + other.right;
    return result;
  }
};



class ArduinoDriverNode : public rclcpp::Node
{
public:
  ArduinoDriverNode() : Node("arduino_driver_node")
  {
    // Create a subscription to the /cmd_vel topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ArduinoDriverNode::twist_callback, this, std::placeholders::_1));
    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(cycle_period_ms), std::bind(&ArduinoDriverNode::send_serial, this));

    // Initialize the serial connection
    serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (serial_port < 0)
    {
      std::cerr << "Error opening serial port: " << strerror(errno) << "\n";
      return;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
      std::cerr << "Error getting terminal attributes: " << strerror(errno) << "\n";
      close(serial_port);
      return;
    }

    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD; // 115200 baud, 8 data bits, no parity, 1 stop bit
    tty.c_iflag = IGNPAR;                         // Ignore framing and parity errors
    tty.c_oflag = 0;                              // Raw output
    tty.c_lflag = 0;                              // Non-canonical input

    if (tcflush(serial_port, TCIFLUSH) != 0)
    {
      std::cerr << "Error flushing serial port: " << strerror(errno) << "\n";
      close(serial_port);
      return;
    }

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
      std::cerr << "Error setting terminal attributes: " << strerror(errno) << "\n";
      close(serial_port);
      return;
    }

    std::cout << "Serial port initialized successfully.\n";
  }

  ~ArduinoDriverNode()
  {
    if (serial_port >= 0)
    {
      std::string command = construct_motor_command(0, 0);
      write(serial_port, command.c_str(), command.size());
      close(serial_port);
      RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_twist = *msg;
  };

  void send_serial()
  {
    // Check if the serial port is open
    if (serial_port <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open!");
      return;
    }

    double linear_velocity = latest_twist.linear.x;   // Linear Velocity (m/s)
    double angular_velocity = latest_twist.angular.z; // Angular Velocity (rad/s)

    double left_speed = linear_velocity - (angular_velocity * wheel_base / 2.0);
    double right_speed = linear_velocity + (angular_velocity * wheel_base / 2.0);

    int32_t left_motor_speed = static_cast<int32_t>(left_speed / wheel_radius * qpp_per_rad);
    int32_t right_motor_speed = static_cast<int32_t>(right_speed / wheel_radius * qpp_per_rad * -1);

    // Construct the command string for the motor controller
    std::string command = construct_motor_command(left_motor_speed, right_motor_speed);

    // Send the command over the serial port
    write(serial_port, command.c_str(), command.size());
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());

    read_serial();
  }

  Robot_Velocity robot_model(const int32_t &left_motor, const int32_t &right_motor)
  {
    Robot_Velocity vel;
    int32_t angular = -(left_motor + right_motor);
    int32_t linear_qp = (left_motor - right_motor) / 2.0;
    vel.omega = angular/(wheel_base/2.0)*qpp_per_meter*2*M_PI; // radians per second
    vel.linear = linear_qp*qpp_per_meter*2*14; // meters per second
    return vel;
  }

  void update_odom()
  {
    time_t current_time = this->now().nanoseconds();
    double delta_time = (current_time - previous_time) / 1e9; // seconds
    if (delta_time <= 0)
    {
      return; // Skip if the time difference is not positive
    }
    if(init == false)
    {
      init = true;
      previous_time = current_time;
      return; // skip the first time
    }

    previous_time = current_time;
    Encoder delta = current_encoder - previous_encoder;

    Robot_Velocity delta_dot = robot_model(delta.left, delta.right);
    
    // Calculate the deltas in x, y, and theta
    double delta_x = delta_dot.linear*cos(current_position.theta) * delta_time;
    double delta_y = delta_dot.linear*sin(current_position.theta) * delta_time;
    double delta_theta = delta_dot.omega * delta_time;

       // Update the current position
    current_position.x += delta_x;
    current_position.y += delta_y;
    current_position.theta += delta_theta;
 
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = current_position.x;
    odom_msg.pose.pose.position.y = current_position.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(current_position.theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(current_position.theta / 2.0);

    odom_msg.twist.twist.linear.x = delta_dot.linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = delta_dot.omega; 
    odom_publisher->publish(odom_msg);
    previous_encoder = current_encoder;  
  }

  void read_serial()
  {
    char buffer[256];
    ssize_t bytes_read;
    bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0)
    {
      buffer[bytes_read] = '\0';
      // Find the ',' and break up the bytes into two integers
      int comma_index = find_index(',', buffer, bytes_read);
      if (comma_index < 0)
      {
        return; // skip there is no comma
      }

      if(comma_index == 0)
      {
        return; // skip if the comma is the first character
      }

      if (comma_index >= bytes_read - 1)
      {
        return; // skip if the comma is the last character
      }

      // Split the buffer into two parts
      char encoder_left_buffer[comma_index + 1]; // comma_index + 1];
      char encoder_right_buffer[bytes_read - comma_index]; // bytes_read - comma_index];
      
      std::copy(buffer, buffer + comma_index, encoder_left_buffer);
      encoder_left_buffer[comma_index] = '\0';

      std::copy(buffer + comma_index + 1, buffer + bytes_read, encoder_right_buffer);
      encoder_right_buffer[bytes_read - comma_index] = '\0';

      int32_t encoder_left = std::stoi(encoder_left_buffer);
      int32_t encoder_right = std::stoi(encoder_right_buffer);

      current_encoder.right = encoder_right;
      current_encoder.left = encoder_left;

      if(init == false)
      {
        initial_encoder = current_encoder;
      }
      else
      {
        current_encoder = current_encoder - initial_encoder;
      }
      update_odom();
    }
    else if (bytes_read == 0)
    {
      // No bytes
    }
  }

  int find_index(const char character, char *array, size_t size)
  {
    for (size_t i = 0; i < size; i++)
    {
      if (array[i] == character)
      {
        return i;
      }
    }
    return -1;
  }

  std::string construct_motor_command(int32_t left_speed, int32_t right_speed)
  {
    return std::to_string(left_speed) + "," + std::to_string(right_speed) + "\n";
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist latest_twist = geometry_msgs::msg::Twist();

  int serial_port;

  double wheel_radius = 32.0 / 1000.0; // meters
  double qpp_per_rad = 1500 / (2 * M_PI);        // qpp per revolution
  double qpp_per_meter = wheel_radius/qpp_per_rad;

  // Convert velocities into left and right motor speeds
  const double wheel_base = 0.150; // Distance between wheels (m)
  bool init = false;
  Encoder initial_encoder = {0,0};
  Encoder previous_encoder = {0,0};
  Encoder current_encoder = {0,0};
  Position current_position = {0,0,0};
  Robot_Velocity current_velocity = {0,0};
  nav_msgs::msg::Odometry odom_msg;
  
  int cycle_period_ms = 50; // milliseconds
  time_t previous_time = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
