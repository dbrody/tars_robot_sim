
#ifndef TARS_CONTROLLER_NN_NEURON
#define TARS_CONTROLLER_NN_NEURON

#ifndef Q_MOC_RUN
	#include <ros/ros.h>
	#include <boost/shared_ptr.hpp>
#endif

#include <QPainter>
#include <QPointF>
#include <vector>

namespace tars_controller_nn
{

class Neuron;

typedef boost::shared_ptr<Neuron> NeuronPtr;

class Neuron {

public:
	Neuron(const ros::NodeHandle& nh, const QPointF& pos);

	bool update();
	double getValue();
	void setValue(double value);
	void paint(QPainter &painter);
	void addInput(NeuronPtr& ptr);

private:
	ros::NodeHandle nh_;

	std::vector<NeuronPtr> inputs_;

	QPointF pos_;

	ros::WallTime last_command_time_;

	double _value;

	// Parameters
	double offset;
	double scale;
};

}

#endif