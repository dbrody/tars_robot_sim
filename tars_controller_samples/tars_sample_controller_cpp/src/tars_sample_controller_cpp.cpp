
#include <ros/ros.h>
#include <tars_control_lib/tars_control_lib.h>
#include <math.h>

int main(int argc, char** argv){
	ROS_INFO("Hello Test.");

	ros::init(argc, argv, "TarsControllerNNApp", ros::init_options::NoSigintHandler);

	ros::NodeHandlePtr nh_;
	nh_.reset(new ros::NodeHandle);
	TarsControlLib::setup(nh_);

	ros::Rate loop_rate(20);

	while(ros::ok()){
		double seconds = ros::Time::now().toSec();

		double value = sin(seconds);

		TarsControlLib::joint1(value);
		TarsControlLib::joint3(-value);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}