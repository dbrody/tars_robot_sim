
#ifndef TARS_CONTROL_LIB_H
#define TARS_CONTROL_LIB_H

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cassert>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>

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
		value = fmod(value, M_PI * 2);
		ROS_INFO("Publishing to Joint1: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint1.publish(msg);
	}

	static bool joint2(double value){
		value = fmod(value, M_PI * 2);
		ROS_INFO("Publishing to Joint2: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint2.publish(msg);
	}

	static bool joint3(double value){
		value = fmod(value, M_PI * 2);
		ROS_INFO("Publishing to Joint3: %f", value);
		std_msgs::Float64 msg;
		msg.data = value;
		get()->pubJoint3.publish(msg);
	}

	static double joint1(){
		return fmod(get()->j1pos, M_PI * 2);
	}

	static double joint2(){
		return fmod(get()->j2pos, M_PI * 2);
	}

	static double joint3(){
		return fmod(get()->j3pos, M_PI * 2);
	}

	static void jointStateCallback(const sensor_msgs::JointState& msg){
		TarsControlLib* inst = get();
		for(unsigned int i = 0; i < msg.name.size(); i++){
			if(msg.name[i].compare("joint1") == 0){
				inst->j1pos = msg.position[i];
				inst->j1vel = msg.velocity[i];
				inst->j1effort = msg.effort[i];
			} else if(msg.name[i].compare("joint2") == 0){
				inst->j2pos = msg.position[i];
				inst->j2vel = msg.velocity[i];
				inst->j2effort = msg.effort[i];
			} else if(msg.name[i].compare("joint3") == 0){
				inst->j3pos = msg.position[i];
				inst->j3vel = msg.velocity[i];
				inst->j3effort = msg.effort[i];
			}
		}
	}

	static bool getModelState(gazebo_msgs::ModelState& modelstate){
		TarsControlLib* inst = get();
		gazebo_msgs::GetModelState srv;
		srv.request.model_name = "tars";
		if(inst->serviceTarsWorldPose.call(srv)){
			ROS_INFO("C++ get model state good.");
			std::cout << typeid(srv.response).name() << std::endl;
			modelstate.model_name = "tars";
			modelstate.pose = srv.response.pose;
			modelstate.twist = srv.response.twist;
			std::cout << srv.response << std::endl;
			return true;
		} else {
			modelstate.model_name = "";
			ROS_INFO("C++ get model state FAIL.");
			return false;
		}
	}

private:
	TarsControlLib(ros::NodeHandlePtr nh){
		ROS_INFO("Created TarControlLib Plugin.");
		nh_ = nh;
		pubJoint1 = nh_->advertise<std_msgs::Float64>("/tars/joint1_position_controller/command", 10);
		pubJoint2 = nh_->advertise<std_msgs::Float64>("/tars/joint2_position_controller/command", 10);
		pubJoint3 = nh_->advertise<std_msgs::Float64>("/tars/joint3_position_controller/command", 10);

		subTarsJointState = nh->subscribe("/tars/joint_states", 10, TarsControlLib::jointStateCallback);

		serviceTarsWorldPose = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	};

	static TarsControlLib* _inst;
	
	ros::NodeHandlePtr nh_;

	TarsControlLib(TarsControlLib const&);
	void operator=(TarsControlLib const&);

	ros::Publisher pubJoint1;
	ros::Publisher pubJoint2;
	ros::Publisher pubJoint3;

	ros::Subscriber subTarsJointState;
	
	ros::ServiceClient serviceTarsWorldPose;

	/* Model Parameters / State */

	// Joint Positions
	double j1pos, j2pos, j3pos;

	// Joint Velocities
	double j1vel, j2vel, j3vel;

	// Joint Effort
	double j1effort, j2effort, j3effort;

};

#endif