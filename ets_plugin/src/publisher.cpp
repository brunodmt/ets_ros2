#include "rclcpp/rclcpp.hpp"
#include "ets_msgs/msg/truck.hpp"

class Publisher : public rclcpp::Node
{
public:
	Publisher() : Node("ets_telemetry")
	{
		publisher_ = this->create_publisher<ets_msgs::msg::Truck>("truck");
	}

	void sendOdometry(float speed, float acc_x, float acc_y, float acc_z, float rpm, int gear, bool engine_running,
			  bool trailer_connected, double x, double y, double z, double heading, double pitch, double roll)
	{
		auto message = ets_msgs::msg::Truck();
		message.speed = speed;
		message.acc_x = acc_x;
		message.acc_y = acc_y;
		message.acc_z = acc_z;
		message.rpm = rpm;
		message.gear = gear;
		message.engine_running = engine_running;
		message.trailer_connected = trailer_connected;
		message.x = x;
		message.y = y;
		message.z = z;
		message.heading = heading;
		message.pitch = pitch;
		message.roll = roll;
		publisher_->publish(message);
	}

private:
	rclcpp::Publisher<ets_msgs::msg::Truck>::SharedPtr publisher_;
};
