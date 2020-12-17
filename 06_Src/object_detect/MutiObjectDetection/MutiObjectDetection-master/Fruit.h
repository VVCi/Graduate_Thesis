/*@Author: NinhNN - CoVV
 *@Date: 2020
 *@Note: Open Source GNU/Linux
*/
#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
using namespace std;
using namespace cv;

/*
 * Define Range:
 * ===/===/===/===/===/===/===/
 *	(1)	(2)	(3)	(4)	(5)	(6)	(7)
 * ===/===/===/===/===/===/===/
*/

#define RANGE_L3	1;
#define RANGE_L2	2;
#define RANGE_L1	3;
#define MID			4;
#define RANGE_R1	5;
#define RANGE_R2	6;
#define RANGE_R3	7;

uint32_t range(uint32_t point);

class Fruit
{
public:
	Fruit();
	~Fruit(void);

	Fruit(string name);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);

	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColour(){
		return Colour;
	}
	void setColour(Scalar c){

		Colour = c;
	}

private:

	int xPos, yPos;
	string type;
	Scalar HSVmin, HSVmax;
	Scalar Colour;
};
