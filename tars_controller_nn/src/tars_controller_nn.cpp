#include <QApplication>

#include <ros/ros.h>
#include <tars_control_lib/tars_control_lib.h>

#include "tars_controller_nn/tars_controller_nn_frame.h"

class TarsControllerNNApp : public QApplication
{
public:
	ros::NodeHandlePtr nh_;

	TarsControllerNNApp(int& argc, char** argv) : QApplication(argc, argv) {
		ros::init(argc, argv, "TarsControllerNNApp", ros::init_options::NoSigintHandler);
		nh_.reset(new ros::NodeHandle);
		TarsControlLib::setup(nh_);
	}

	int exec() {
		tars_controller_nn::TarsControllerNNFrame frame;
		frame.show();
		return QApplication::exec();
	}
};

int main(int argc, char** argv){
	TarsControllerNNApp app(argc, argv);
	return app.exec();
}