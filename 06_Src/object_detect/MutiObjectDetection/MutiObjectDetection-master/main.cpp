#include "mainwindow.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <QFileDialog>
#include <QLabel>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <QTimerEvent>
#include <zconf.h>
#include <zlib.h>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QMainWindow>
#include <iostream>
using namespace std;
using namespace cv;

#include "Fruit.h"
#include "serial.h"
#include "distance.h"

/*
 * Initial min and max HSV filter values
 * these will be changed using trackbars
 */
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 70;
int S_MAX = 145;
int V_MIN = 220;
int V_MAX = 256;

/* Point */
uint32_t posX, posY, postXReturn, postYReturn;

/* Distance */
uint32_t gDistance;

/* Buffer Data Transfer */
uint8_t *data = NULL;
uint8_t send_data[2];

/* Default capture width and height */
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

/* Max number of objects to be detected in frame */
const int MAX_NUM_OBJECTS = 50;

/* Minimum and maximum object area */
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

/* Names that will appear at the top of each window */
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

/*
 * This function gets called whenever a
 *  trackbar position is changed
*/
void on_trackbar( int, void* ){
  /* do nothing */
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){
	/* Create window for trackbars */
	namedWindow(trackbarWindowName,0);

	/* Create memory to store trackbar name on window */
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);

	/*
	 *  Create trackbars and insert them into window
	 * 3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	 * the max value the trackbar can move (eg. H_HIGH),
	 * and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	 *  ---->    ---->     ---->
	*/
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

void drawObject(vector<Fruit> theFruits,Mat &frame){
	for(int i =0; i < theFruits.size(); i++){
		cv::circle(frame,cv::Point(theFruits.at(i).getXPos(),theFruits.at(i).getYPos()),10,cv::Scalar(0,0,255));
		cv::putText(frame,intToString(theFruits.at(i).getXPos())+ " , " + intToString(theFruits.at(i).getYPos()),cv::Point(theFruits.at(i).getXPos(),theFruits.at(i).getYPos()+20),1,1,Scalar(0,255,0));
		cv::putText(frame,theFruits.at(i).getType(),cv::Point(theFruits.at(i).getXPos(),theFruits.at(i).getYPos()-30),1,2,theFruits.at(i).getColour());

		/* Sending X location via serial */
		posX = (theFruits.at(i).getXPos());
		postXReturn = range(posX);
		send_data[0] = postXReturn;
		send_data[1] = 0;
		tx_frame_post((uint8_t *)send_data, 2);
	}
}

void morphOps(Mat &thresh){
	/*
	 * Create structuring element that will be used to "dilate" and "erode" image.
	 * the element chosen here is a 3px by 3px rectangle
	*/
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));

	/* Dilate with larger element so make sure object is nicely visible */
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed){
	vector <Fruit> apples;

	Mat temp;
	threshold.copyTo(temp);

	/* These two vectors needed for output of findContours */
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/* Find contours of filtered image using openCV findContours function */
	findContours(temp, contours, hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	/* Use moments method to find our filtered object */
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		/* If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter */
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				/* If the area is less than 20 px by 20px then it is probably just noise
				 * if the area is the same as the 3/2 of the image size, probably just a bad filter
				 * we only want the object with the largest area so we safe a reference area each
				 * iteration and compare it to the area in the next iteration.
				*/
				if(area>MIN_OBJECT_AREA){
					Fruit apple;
					apple.setXPos(moment.m01/area);
					apple.setYPos(moment.m10/area);
					apples.push_back(apple);
					objectFound = true;
				}else objectFound = false;
			}

			/* Let user know you found an object */
			if(objectFound ==true){
				/* Draw object location on screen */
				drawObject(apples,cameraFeed);}
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

void trackFilteredObject(Fruit theFruit,Mat threshold,Mat HSV, Mat &cameraFeed){
	vector <Fruit> apples;
	Mat temp;
	threshold.copyTo(temp);
	/* These two vectors needed for output of findContours */
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/* Find contours of filtered image using openCV findContours function */
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	/* Use moments method to find our filtered object */
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		/* If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter */
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				/* If the area is less than 20 px by 20px then it is probably just noise
				 * If the area is the same as the 3/2 of the image size, probably just a bad filter
				 * we only want the object with the largest area so we safe a reference area each
				 * iteration and compare it to the area in the next iteration. */
				if(area>MIN_OBJECT_AREA){
					Fruit apple;
					apple.setXPos(moment.m10/area);
					apple.setYPos(moment.m01/area);
					apple.setType(theFruit.getType());
					apple.setColour(theFruit.getColour());
					apples.push_back(apple);
					objectFound = true;
				}else objectFound = false;
			}

			/* Let user know you found an object */
			if(objectFound ==true){
				/* Draw object location on screen */
				drawObject(apples,cameraFeed);}
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

int main(int argc, char *argv[]){

	QApplication a(argc, argv);
	MainWindow w;
	/* If we would like to calibrate our filter values, set to true */
	bool calibrationMode = true;

	/* Matrix to store each frame of the webcam feed */
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;

	if(calibrationMode){
		/* Create slider bars for HSV filtering */
		createTrackbars();

		/* Open Serial */
		if (if_cpu_serial_opentty("/dev/ttyACM0") == 0) {
			qDebug("Open serial OK\n");
		}
		else {
			qDebug("Open serial ERROR\n");
		}
	}

	/* Video capture object to acquire webcam feed */
	VideoCapture capture;

	/* Open capture object at location zero (default location for webcam) */
	capture.open(0);

	/* Set height and width of capture frame */
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

	/* Start an infinite loop where webcam feed is copied to cameraFeed matrix
	 * all of our operations will be performed within this loop */
	while(1){
		/* Store image to matrix */
		capture.read(cameraFeed);

		/* Convert frame from BGR to HSV colorspace */
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

		if(calibrationMode==true){
			/* If in calibration mode, we track objects based on the HSV slider values */
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
			morphOps(threshold);
			imshow(windowName2,threshold);
			trackFilteredObject(threshold,HSV,cameraFeed);
		}else{
			/*
			 * create some temp fruit objects so that
			 * we can use their member functions/information
			*/
			Fruit apple("apple"), banana("banana"), cherry("cherry");
			/* First find apples */
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,apple.getHSVmin(),apple.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(apple,threshold,HSV,cameraFeed);
			/* Then bananas */
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,banana.getHSVmin(),banana.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(banana,threshold,HSV,cameraFeed);
			/* Then cherries */
			cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,cherry.getHSVmin(),cherry.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(cherry,threshold,HSV,cameraFeed);
		}

		/* Show frames */
		//imshow(windowName2,threshold);
		imshow(windowName,cameraFeed);
		//imshow(windowName1,HSV);

		/*
		 * delay 30ms so that screen can refresh
		 * image will not appear without this waitKey() command
		*/
		waitKey(30);
	}
	w.show();
	return a.exec();
}
