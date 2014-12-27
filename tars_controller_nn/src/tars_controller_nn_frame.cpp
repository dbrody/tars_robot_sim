
#include "tars_controller_nn/tars_controller_nn_frame.h"

#include <QPointF>
#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xFF

namespace tars_controller_nn
{

TarsControllerNNFrame::TarsControllerNNFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, frame_count_(0)
, id_counter_(0)
{
	setFixedSize(500, 500);
	setWindowTitle("Tars Controller NN Display");

	srand(time(NULL));

	update_timer_ = new QTimer(this);
	update_timer_->setInterval(16);
	update_timer_->start();

	connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

	nh_.setParam("background_r", DEFAULT_BG_R);
	nh_.setParam("background_g", DEFAULT_BG_G);
	nh_.setParam("background_b", DEFAULT_BG_B);

	clear();

	ROS_INFO("Starting TarsControllerNN with node name %s", ros::this_node::getName().c_str());

	NeuronPtr in1 = makeInputNeuron(0, width() / 2 - 100, height() / 2 + 100);
	NeuronPtr in2 = makeInputNeuron(1, width() / 2, height() / 2 + 100);
	NeuronPtr in3 = makeInputNeuron(2, width() / 2 + 100, height() / 2 + 100);

	NeuronPtr hid1 = makeHiddenNeuron(0, width() / 2 - 100, height() / 2);
	NeuronPtr hid2 = makeHiddenNeuron(1, width() / 2, height() / 2);
	NeuronPtr hid3 = makeHiddenNeuron(2, width() / 2 + 100, height() / 2);
	hid1->addInput(in1);
	hid1->addInput(in2);
	
	hid2->addInput(in2);
	hid3->addInput(in3);

	NeuronPtr out1 = makeOutputNeuron(0, width() / 2 - 100, height() / 2 - 100);
	NeuronPtr out2 = makeOutputNeuron(1, width() / 2, height() / 2 - 100);
	NeuronPtr out3 = makeOutputNeuron(2, width() / 2 + 100, height() / 2 - 100);
	out1->addInput(hid1);
	out2->addInput(hid2);
	out3->addInput(hid3);
}

TarsControllerNNFrame::~TarsControllerNNFrame(){
	delete update_timer_;
}

NeuronPtr TarsControllerNNFrame::makeInputNeuron(int id, float x, float y){
	NeuronPtr n(new Neuron(ros::NodeHandle("neuron"), QPointF(x, y)));
	neurons_input_[id] = n;
	update();

	ROS_INFO("Spawning Neuron [%d] as x=[%f], y=[%f]", id, x, y);

	return n;
}

NeuronPtr TarsControllerNNFrame::makeOutputNeuron(int id, float x, float y){
	NeuronPtr n(new Neuron(ros::NodeHandle("neuron"), QPointF(x, y)));
	neurons_output_[id] = n;
	update();

	ROS_INFO("Spawning Neuron [%d] as x=[%f], y=[%f]", id, x, y);

	return n;
}

NeuronPtr TarsControllerNNFrame::makeHiddenNeuron(int id, float x, float y){
	NeuronPtr n(new Neuron(ros::NodeHandle("neuron"), QPointF(x, y)));
	neurons_hidden_[id] = n;
	update();

	ROS_INFO("Spawning Neuron [%d] as x=[%f], y=[%f]", id, x, y);

	return n;
}

void TarsControllerNNFrame::clear(){
	int r = DEFAULT_BG_R;
	int g = DEFAULT_BG_G;
	int b = DEFAULT_BG_B;

	nh_.param("background_r", r, r);
	nh_.param("background_g", g, g);
	nh_.param("background_b", b, b);

	update();
}

void TarsControllerNNFrame::onUpdate()
{
	ROS_INFO("Updating TarsControllerNN");

	ros::spinOnce();

	updateNeurons();

	if(!ros::ok()){
		close();
	}
}

void TarsControllerNNFrame::paintEvent(QPaintEvent* event){

	double now = ros::Time::now().toSec();

	double j1 = TarsControlLib::joint1();
	double j2 = TarsControlLib::joint2();
	double j3 = TarsControlLib::joint3();

	double value = j1 + 0.1;
	ROS_INFO("Sending Now: %.2f -> %.2f", j1, value);
	

	neurons_input_[0]->setValue(j1);
	neurons_input_[1]->setValue(j2);
	neurons_input_[2]->setValue(j3);

	QPainter painter(this);
	M_Neurons::iterator itIn = neurons_input_.begin();
	M_Neurons::iterator endIn = neurons_input_.end();
	for(; itIn != endIn; ++itIn){
		itIn->second->paint(painter);
	}

	M_Neurons::iterator itHidden = neurons_hidden_.begin();
	M_Neurons::iterator endHidden = neurons_hidden_.end();
	for(; itHidden != endHidden; ++itHidden){
		itHidden->second->update();
		itHidden->second->paint(painter);
	}

	M_Neurons::iterator itOut = neurons_output_.begin();
	M_Neurons::iterator endOut = neurons_output_.end();
	for(; itOut != endOut; ++itOut){
		itOut->second->update();
		itOut->second->paint(painter);
	}

	TarsControlLib::joint1(neurons_output_[0]->getValue());
	TarsControlLib::joint2(neurons_output_[1]->getValue());
	TarsControlLib::joint3(neurons_output_[2]->getValue());
}

void TarsControllerNNFrame::updateNeurons(){
	if(last_neuron_update_.isZero())
	{
		last_neuron_update_ = ros::WallTime::now();
		return;
	}

	// TODO
	update();
}




}
