#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFQueryTester : public rclcpp::Node
{
public:
  TFQueryTester()
  : Node("tf_query_tester"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TFQueryTester::query_transform, this));
  }

private:
  void query_transform()
  {
    rclcpp::Time now = this->get_clock()->now();
    try
    {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform("odom", "base_link", now);
        rclcpp::Time now = this->get_clock()->now();
        int64_t total_ns = now.nanoseconds();
        long sec = total_ns / 1000000000LL;
        long nsec = total_ns % 1000000000LL;
        RCLCPP_INFO(
          this->get_logger(),
          "Transform at time %ld.%09ld: [x: %f, y: %f, z: %f]",
          sec,
          nsec,
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFQueryTester>());
  rclcpp::shutdown();
  return 0;
}
