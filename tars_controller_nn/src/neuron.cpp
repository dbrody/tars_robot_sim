
#include "tars_controller_nn/neuron.h"

#include <algorithm>
#include <stdlib.h>

#include <QBrush>
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
	offset = (rand() % 1000) / 1000.0 - 0.5;
	scale = (rand() % 1000) / 1000.0 + 0.5;
}

bool Neuron::update(){
	double total = 0;
	for(unsigned int i = 0; i < inputs_.size(); i++){
		total += inputs_[i]->getValue();
	}

	total -= offset;
	total *= scale;
	total = std::max(0.0, total);
	double value = total / (1 + abs(total));
	setValue(value);
	return true;
}

void Neuron::paint(QPainter& painter)
{
	QFont font;
	font.setPixelSize(10);

	// Draw cricle
	int color = 255 - (_value / 10) * 255;
	if(_value < 0){
		color = 255;
	}
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
}

void Neuron::setValue(double value){
	_value = value;
}

double Neuron::getValue(){
	return _value;
}


}