
#ifndef TARS_CONTROL_LIB_H
#define TARS_CONTROL_LIB_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cassert>

class TarsControlLib {

public:

	static void setup(ros::NodeHandlePtr nh){
		if(_inst == NULL){
			_inst = new TarsControlLib(nh);
		}
	}

	static TarsControlLib* get(){
		assert(_inst != NULL);
		return _inst;
	}

	static bool joint1(double value){
		ROS_INFO("Publishing to Joint1: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint1.publish(msg);
	}

	static bool joint2(double value){
		ROS_INFO("Publishing to Joint2: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint2.publish(msg);
	}

	static bool joint3(double value){
		ROS_INFO("Publishing to Joint3: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint3.publish(msg);
	}

private:
	TarsControlLib(ros::NodeHandlePtr nh){
		ROS_INFO("Created TarControlLib Plugin.");
		nh_ = nh;
		pubJoint1 = nh_->advertise<std_msgs::Float64>("/tars/joint1_position_controller/command", 10);
		pubJoint2 = nh_->advertise<std_msgs::Float64>("/tars/joint2_position_controller/command", 10);
		pubJoint3 = nh_->advertise<std_msgs::Float64>("/tars/joint3_position_controller/command", 10);
	};

	static TarsControlLib* _inst;
	
	ros::NodeHandlePtr nh_;

	TarsControlLib(TarsControlLib const&);
	void operator=(TarsControlLib const&);

	ros::Publisher pubJoint1;
	ros::Publisher pubJoint2;
	ros::Publisher pubJoint3;
};

#endif