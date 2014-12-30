
#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

#include <tars_control_lib/tars_control_lib.h>
#include <gazebo_msgs/ModelState.h>


#ifndef Q_MOC_RUN
	#include <ros/ros.h>
	#include <std_srvs/Empty.h>
	#include <map>

	#include "neuron.h"
#endif

namespace tars_controller_nn
{

class TarsControllerNNFrame : public QFrame
{

	Q_OBJECT
public:
	TarsControllerNNFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
	~TarsControllerNNFrame();

	NeuronPtr makeInputNeuron(int id, float x, float y);
	NeuronPtr makeOutputNeuron(int id, float x, float y);
	NeuronPtr makeHiddenNeuron(int id, float x, float y);

protected:
	void paintEvent(QPaintEvent* event);

private slots:
	void onUpdate();

private:
	void updateNeurons();
	void clear();

	double start_time;

	ros::NodeHandle nh_;
	QTimer* update_timer_;

	uint64_t frame_count_;

	ros::WallTime last_neuron_update_;

	typedef std::map<int, NeuronPtr> M_Neurons;
	M_Neurons neurons_input_;
	M_Neurons neurons_hidden_;
	M_Neurons neurons_output_;
	uint32_t id_counter_;

	float meter_;
	float width_in_meters_;
	float height_in_meters_;
};

}