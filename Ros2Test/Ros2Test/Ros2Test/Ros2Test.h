#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;


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
		: Node("ros2_unity_node")
	{
		state = Vector2f{ 1,2 };
		message_ = nav_msgs::msg::Odometry();
		//message_.header.stamp = rclcpp::Node::now(); // this->now();
		message_.header.frame_id = "odom";
		message_.child_frame_id = "base";
		_parse_state_to_message(state);
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
		node_ = std::make_shared<CustomNode>();
		rclcpp::spin_some(node_);
		//thread
		//rclcpp::ExecutorOptions options();
		//executor_->add_node(node_);
		//executor_.spin_some();
	}
	//rclcpp::executors::MultiThreadedExecutor* executor_;
	std::shared_ptr<CustomNode> node_;
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

};


// define c API
#define DLLExport __declspec(dllexport)

extern "C" {
	DLLExport Wrapper* initialiseObject();
	DLLExport void update(Wrapper*);
	DLLExport void update_state(Wrapper*, Vector2f state);
	DLLExport void start_publish(Wrapper*, Vector2f);
}
