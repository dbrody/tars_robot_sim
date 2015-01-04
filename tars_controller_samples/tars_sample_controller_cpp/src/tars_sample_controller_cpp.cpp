
#include <ros/ros.h>
#include <tars_corecpp/tars_corecpp.h>
#include <math.h>

int main(int argc, char** argv){
	ROS_INFO("Hello Test.");

	if(argc < 2){
		ROS_INFO("Required [name] arg.");
		exit(1);
	}

	ros::init(argc, argv, "TarsControllerNNApp", ros::init_options::NoSigintHandler);

	ros::NodeHandlePtr nh_;
	nh_.reset(new ros::NodeHandle);
	TarsCore::setup(nh_, "TarsControllerNNApp");

	ros::Rate loop_rate(20);

	while(ros::ok()){
		double seconds = ros::Time::now().toSec();

		double value = sin(seconds);

		TarsCore::joint1(value);
		TarsCore::joint3(-value);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}