#include "rclcpp/rclcpp.hpp"
#include "ets_msgs/msg/truck.hpp"

class Publisher : public rclcpp::Node
{
public:
	Publisher() : Node("ets_telemetry")
	{
		publisher_ = this->create_publisher<ets_msgs::msg::Truck>("truck");
	}

	void sendOdometry(float speed, float rpm, int gear, bool trailer_connected)
	{
		auto message = ets_msgs::msg::Truck();
		message.speed = speed;
		message.rpm = rpm;
		message.gear = gear;
		message.trailer_connected = trailer_connected;
		publisher_->publish(message);
	}

private:
	rclcpp::Publisher<ets_msgs::msg::Truck>::SharedPtr publisher_;
};
