#include "Fruit.h"
#include <QApplication>


Fruit::Fruit()
{
	//set values for default constructor
	setType("Point");
	setColour(Scalar(0,0,0));
}

Fruit::Fruit(string name){
	setType(name);
	
	if(name=="apple"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(0,0,0));
		setHSVmax(Scalar(255,255,255));

		//BGR value for Green:
		setColour(Scalar(0,255,0));

	}
	if(name=="banana"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(0,0,0));
		setHSVmax(Scalar(255,255,255));

		//BGR value for Yellow:
		setColour(Scalar(0,255,255));

	}
	if(name=="cherry"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(0,0,0));
		setHSVmax(Scalar(255,255,255));

		//BGR value for Red:
		setColour(Scalar(0,0,255));

	}

}

Fruit::~Fruit(void)
{
}

int Fruit::getXPos(){

	return Fruit::xPos;

}

void Fruit::setXPos(int x){

	Fruit::xPos = x;

}

int Fruit::getYPos(){

	return Fruit::yPos;

}

void Fruit::setYPos(int y){

	Fruit::yPos = y;

}

Scalar Fruit::getHSVmin(){

	return Fruit::HSVmin;

}
Scalar Fruit::getHSVmax(){

	return Fruit::HSVmax;
}

void Fruit::setHSVmin(Scalar min){

	Fruit::HSVmin = min;
}


void Fruit::setHSVmax(Scalar max){

	Fruit::HSVmax = max;
}

uint32_t range(uint32_t point){

	/* Turn Left_3*/
	if (point >= 0 && point < 91){
		qDebug("1");
		return RANGE_L3;
	}

	/* Turn Left_2*/
	else if (point >= 91 && point < 182){
		qDebug("2");
		return RANGE_L2;
	}

	/* Turn Left_1*/
	else if (point >= 182 && point < 273){
		qDebug("3");
		return RANGE_L1;
	}

	/* Hold Center*/
	else if (point >= 273 && point < 367){
		qDebug("4");
		return MID;
	}

	/* Turn Right_1*/
	else if (point >= 367 && point < 458){
		qDebug("5");
		return RANGE_R1;
	}

	/* Turn Right_2*/
	else if (point >= 458 && point < 549){
		qDebug("6");
		return RANGE_R2;
	}

	/* Turn Right_3*/
	else if (point >= 549 && point < 640){
		qDebug("7");
		return RANGE_R3;
	}

	/* Hold Center*/
	else {
		qDebug("4");
		return MID;
	}
}
