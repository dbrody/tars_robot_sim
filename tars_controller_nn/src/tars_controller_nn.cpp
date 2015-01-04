#include <QApplication>

#include <ros/ros.h>
#include <string.h>
#include <tars_corecpp/tars_corecpp.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include "tars_controller_nn/tars_controller_nn_frame.h"

void sigint_handler(int sig){
	ros::shutdown();
}

class TarsControllerNNApp : public QApplication
{
public:
	ros::NodeHandlePtr nh_;

	TarsControllerNNApp(int& argc, char** argv) : QApplication(argc, argv) {
		char name[128] = "TarsControllerNNApp";
		if(argc > 1){
			strcpy(name, argv[1]);
		}
		ros::init(argc, argv, name);
		
		// Handle interrupts
		signal(SIGINT, sigint_handler);

		nh_.reset(new ros::NodeHandle);
		TarsCore::setup(nh_, name);
	}

	int exec() {
		tars_controller_nn::TarsControllerNNFrame frame;
		frame.show();
		return QApplication::exec();
	}
};

int main(int argc, char** argv){
	timeval t1;
	gettimeofday(&t1, NULL);
	srand(t1.tv_usec * t1.tv_sec);
	TarsControllerNNApp app(argc, argv);
	return app.exec();
}