
#include "tars_controller_nn/neuron.h"

#include <algorithm>
#include <stdlib.h>
#include <math.h>

#include <QBrush>
#include <QPen>
#include <QColor>
#include <QRgb>
#include <QFont>

namespace tars_controller_nn
{

Neuron::Neuron(const ros::NodeHandle& nh, const QPointF& pos)
: nh_(nh)
, pos_(pos)
, _value(0)
{
	offset = (rand() % 20000) / 10000.0 - 1;
}

bool Neuron::update(){
	double total = 0;
	double total_weight = 0;
	for(unsigned int i = 0; i < inputs_.size(); i++){
		total_weight += fabs(weights_[i]);
		total += weights_[i] * inputs_[i]->getValue();
	}
	total /= total_weight;
	total -= offset;
	double value = total;
	//double value = 1.0 / (1.0 + exp(-total));
	setValue(value);
	return true;
}

void Neuron::paint(QPainter& painter)
{
	QFont font;
	font.setPixelSize(10);

	// Draw Inputs
	for(unsigned int i = 0; i < inputs_.size(); i++){
		int color = 255 * fabs(weights_[i]);
		if(weights_[i] > 0){
			painter.setPen(QPen(QColor(color, color, color)));
		} else {
			painter.setPen(QPen(QColor(color, 0, 0)));
		}
		painter.drawLine(inputs_[i]->getPos(), pos_);
	}

	painter.setPen(QPen(QColor(0, 0, 0)));

	// Draw cricle
	int color = 255;
	painter.setBrush(QBrush(QColor(color, color, color)));
	painter.drawEllipse(pos_, 20, 20);

	// Draw activation value
	int stringColor = (color > 128) ? 0 : 255;
	painter.setBrush(QBrush(QColor(stringColor, stringColor, stringColor)));
	QString str = QString::number((double)_value, 'd', 2);
	painter.setFont(font);
	painter.drawText(pos_.x() - 7, pos_.y(), str);


}

void Neuron::addInput(NeuronPtr& ptr){
	inputs_.push_back(ptr);
	weights_.push_back((rand() % 20000) / 10000.0 - 1.0);
}

void Neuron::setValue(double value){
	_value = value;
}

double Neuron::getValue(){
	return _value;
}

QPointF Neuron::getPos(){
	return pos_;
}


}