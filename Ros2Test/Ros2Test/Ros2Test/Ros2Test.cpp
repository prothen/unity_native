// example 
//			https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/#cpppubsub


#include "pch.h"
#include "framework.h"

#undef max
#undef min
#include "Ros2Test.h"


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