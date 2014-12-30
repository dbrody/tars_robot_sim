
#include "tars_controller_nn/tars_controller_nn_frame.h"

#include <QPointF>
#include <ros/package.h>
#include <cstdlib>
#include <ctime>
#include <math.h>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xFF

namespace tars_controller_nn
{

TarsControllerNNFrame::TarsControllerNNFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, frame_count_(0)
, id_counter_(0)
, start_time(0)
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

	int numIn = 8;
	int numHidden = 3;
	int numOut = 3;
	int spaceBetween = 70;
	
	for(int i = 0; i < numIn; i++){
		int offset = (i - numIn / 2) * spaceBetween;
		makeInputNeuron(i, width() / 2 + offset, height() / 2 + 100);		
	}

	for(int i = 0; i < numHidden; i++){
		int offset = (i - numHidden / 2) * spaceBetween;
		makeHiddenNeuron(i, width() / 2 + offset, height() / 2);		
	}

	for(int i = 0; i < numOut; i++){
		int offset = (i - numOut / 2) * spaceBetween;
		makeOutputNeuron(i, width() / 2 + offset, height() / 2 - 100);		
	}
	
	// Connect all hidden to input
	M_Neurons::iterator itHidden = neurons_hidden_.begin();
	M_Neurons::iterator endHidden = neurons_hidden_.end();
	for(; itHidden != endHidden; ++itHidden){
		M_Neurons::iterator itIn = neurons_input_.begin();
		M_Neurons::iterator endIn = neurons_input_.end();
		for(; itIn != endIn; ++itIn){
			itHidden->second->addInput(itIn->second);
		}
	}

	// Connect all output to hidden
	M_Neurons::iterator itOut = neurons_output_.begin();
	M_Neurons::iterator endOut = neurons_output_.end();
	for(; itOut != endOut; ++itOut){
		M_Neurons::iterator itHidden2 = neurons_hidden_.begin();
		M_Neurons::iterator endHidden2 = neurons_hidden_.end();
		for(; itHidden2 != endHidden2; ++itHidden2){
			itOut->second->addInput(itHidden2->second);
		}
	}

	start_time = ros::Time::now().toSec();
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

	ROS_INFO("Start: %.2f -> %.2f", start_time, now);
	if(start_time == 0 || now - start_time < 3){
		if(start_time == 0){
			start_time = now;
		}
		return;
	}

	double j1 = TarsControlLib::joint1();
	double j2 = TarsControlLib::joint2();
	double j3 = TarsControlLib::joint3();

	gazebo_msgs::ModelState modelstate;
	TarsControlLib::getModelState(modelstate);
	double o1 = modelstate.pose.orientation.x;
	double o2 = modelstate.pose.orientation.y;
	double o3 = modelstate.pose.orientation.z;

	// Set Input Neuron Values
	neurons_input_[0]->setValue(j1);
	neurons_input_[1]->setValue(j2);
	neurons_input_[2]->setValue(j3);

	neurons_input_[3]->setValue(o1);
	neurons_input_[4]->setValue(o2);
	neurons_input_[5]->setValue(o3);

	neurons_input_[6]->setValue(sin(3 * now));
	neurons_input_[7]->setValue(cos(3 * now));

	// Set up iterators
	M_Neurons::iterator itIn = neurons_input_.begin();
	M_Neurons::iterator endIn = neurons_input_.end();

	M_Neurons::iterator itHidden = neurons_hidden_.begin();
	M_Neurons::iterator endHidden = neurons_hidden_.end();

	M_Neurons::iterator itOut = neurons_output_.begin();
	M_Neurons::iterator endOut = neurons_output_.end();

	// Update neurons (hidden then output)
	for(; itHidden != endHidden; ++itHidden) itHidden->second->update();
	for(; itOut != endOut; ++itOut) itOut->second->update();

	// Reset iterators
	itIn = neurons_input_.begin();
	endIn = neurons_input_.end();

	itHidden = neurons_hidden_.begin();
	endHidden = neurons_hidden_.end();

	itOut = neurons_output_.begin();
	endOut = neurons_output_.end();
	
	// Draw
	QPainter painter(this);
	for(; itOut != endOut; ++itOut) itOut->second->paint(painter);
	for(; itHidden != endHidden; ++itHidden) itHidden->second->paint(painter);
	for(; itIn != endIn; ++itIn) itIn->second->paint(painter);

	// Send output values to joint commands
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
