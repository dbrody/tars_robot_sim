# tars_control_lib

C++ shared library for controlling the TARS robot during simulation.

## Usage in code

This library is created as a **singleton**. Set it up once and then interact with it from anywhere.

Include the header file within any c++ file:

	#include <tars_control_lib>

Initiate with a reference to the ros::NodeHandlePtr so it can initialize the publishing topics:

	TarsControlLib::setup(nh);

From then on you may send commands to the robot:

	TarsControlLib::joint1((double)0.1);
	TarsControlLib::joint2(sin(i));
	TarsControlLib::joint3(ros::Time::now().toSec());

## Usage in Packages

Be sure to add a dependency on tars_control_lib in a package.xml:
	
	<build_depend>tars_control_lib</build_depend>
	<run_depend>tars_control_lib</run_depend>

Also add dependency on package in CMakeListd.txt:

	find_package(catkin REQUIRED COMPONENTS
		...
		tars_control_lib
		...
	)