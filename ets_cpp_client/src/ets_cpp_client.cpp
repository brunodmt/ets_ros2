#include "rclcpp/rclcpp.hpp"
#include "ets_msgs/msg/truck.hpp"

void callback(const ets_msgs::msg::Truck::SharedPtr msg)
{
  std::cout << "speed=" << msg->speed << " rpms=" << msg->rpm << " gear=" << msg->gear << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ets_client");

  auto sub = node->create_subscription<ets_msgs::msg::Truck>(
    "truck", callback, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}