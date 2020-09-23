#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include<iostream>

using namespace std::chrono_literals;

/*
 * Todo:
		- investigate build failure in Unity3d -> causes runtime error on rclcpp::init
 *
 */

struct Vector2f {
	double x;
	double y;
};

struct DecoupledState {
	Vector2f position;
	Vector2f velocity;
};

struct Control {
	Vector2f acceleration;
};


class CustomNode : public rclcpp::Node
{
public:
	Vector2f state;
	CustomNode()
		: Node("ros2_unity_node"), count_{0}
	{
		state = Vector2f{ 1,2 };
		message_ = nav_msgs::msg::Odometry();
		message_.header.stamp = rclcpp::Node::now(); // this->now();
		message_.header.frame_id = "odom";
		message_.child_frame_id = "base";
		_parse_state_to_message(state);
		//start_publish();
	}
	void start_publish() {
		publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("groundtruth", 10);
		timer_ = this->create_wall_timer(500ms, std::bind(&CustomNode::_timer_callback, this));
	}
	void update_message(Vector2f& state) {
		_parse_state_to_message(state);
	}
private:
	void _parse_state_to_message(Vector2f& state) {
		message_.pose.pose.position.x = state.x;
		message_.pose.pose.position.y = state.y;
	}

	void _timer_callback()
	{
		message_.header.stamp = this->now(); // rclcpp::Node::now();
											 //message_.header.stamp = this->get_clock->now();
											 //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.pose.pose.position.x);
		RCLCPP_INFO(this->get_logger(), "%i - Publishing: ", count_);
		publisher_->publish(message_);
		count_++;
	}
	nav_msgs::msg::Odometry message_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

	size_t count_;
};

class Wrapper {
public:
	Wrapper(int argc, char* argv[]) {
		rclcpp::init(argc, argv);
		// node_ = std::make_shared<CustomNode>();
		//rclcpp::spin_some(node_);
		//thread
		//rclcpp::ExecutorOptions options();
		//executor_->add_node(node_);
		//executor_.spin_some();
	}
	//rclcpp::executors::MultiThreadedExecutor* executor_;
	void start_publish() {
		node_->start_publish();
	}
	void update() {
		rclcpp::spin_some(node_);
	}

	void spin() {
		rclcpp::spin(node_);
	}
	void update_state(Vector2f state) {
		node_->update_message(state);
	}
private:
	std::shared_ptr<CustomNode> node_;
};


// define c API
#define DLLExport __declspec(dllexport)

extern "C" {
	DLLExport Wrapper* initialiseObject();
	DLLExport void update(Wrapper*);
	DLLExport void update_state(Wrapper*, Vector2f state);
	DLLExport void start_publish(Wrapper*, Vector2f);
}
//
//
Wrapper* initialiseObject() {
	Wrapper* ros2_interface = new Wrapper(0, nullptr);
	return ros2_interface;
}


void update(Wrapper* ros2_interface) {
	ros2_interface->update();
}

void update_state(Wrapper* ros2_interface, Vector2f state) {
	ros2_interface->update_state(state);
}

void start_publish(Wrapper* ros2_interface) {
	ros2_interface->start_publish();
}

std::ostream& operator<<(std::ostream& os, const Vector2f& state) {
	return os << "\tX: " << state.x << std::endl
		<< "\tY: " << state.y << std::endl;
}
std::ostream& operator<<(std::ostream& os, const DecoupledState& state) {
	return os << "Position: " << std::endl
		<< state.position
		<< "Velocity: " << std::endl
		<< state.velocity;
}
//int main(int argc, char* argv[])
//{
//	int n;
//	Vector2f position = { 0, 1 };
//	DecoupledState test = { position,{ 2,3 } };
//	std::cout << "Test State:\n" << test << std::endl;
//	std::cout << "Start ROS2 node" << std::endl;
//	Wrapper* ros2_interface = new Wrapper(argc, argv);
//	ros2_interface->start_publish();
//	ros2_interface->spin();
//	std::cout << "Started ROS2 node" << std::endl;
//	std::cin >> n;
//	return 0;
//}