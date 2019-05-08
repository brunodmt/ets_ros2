// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define NOMINMAX

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
		: Node("ets_telemetry")
	{
		publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom");
	}

	void sendOdometry(float speed, float x, float y, float z, float heading, float pitch, float roll)
	{
		auto message = nav_msgs::msg::Odometry();
		message.pose.pose.position.x = x;
		message.pose.pose.position.y = y;
		message.pose.pose.position.z = z;
		message.pose.pose.orientation.x = heading;
		message.pose.pose.orientation.y = pitch;
		message.pose.pose.orientation.z = roll;

		message.twist.twist.linear.x = speed;
		publisher_->publish(message);
	}

private:

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};